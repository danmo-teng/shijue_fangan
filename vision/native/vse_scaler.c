#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include <hb_mem_mgr.h>
#include <hbn_api.h>
#include <vse_cfg.h>

typedef struct {
    hbn_vnode_handle_t vnode;
    hbn_vnode_image_t input;
    int input_width;
    int input_height;
    int output_width;
    int output_height;
    uint32_t frame_id;
    int memory_open;
    int vnode_open;
    int started;
} rescue_vse_scaler_t;

static void destroy_scaler(rescue_vse_scaler_t *scaler)
{
    if (scaler == NULL) return;
    if (scaler->started) {
        hbn_vnode_stop(scaler->vnode);
        scaler->started = 0;
    }
    if (scaler->vnode_open) {
        hbn_vnode_close(scaler->vnode);
        scaler->vnode_open = 0;
    }
    if (scaler->input.buffer.fd[0] >= 0) {
        hb_mem_free_buf(scaler->input.buffer.fd[0]);
        scaler->input.buffer.fd[0] = -1;
    }
    if (scaler->memory_open) {
        hb_mem_module_close();
        scaler->memory_open = 0;
    }
}

void *rescue_vse_create(int input_width, int input_height,
                        int output_width, int output_height)
{
    if (input_width <= 0 || input_height <= 0 || output_width <= 0 || output_height <= 0 ||
        (input_width & 1) || (input_height & 1) || (output_width & 1) || (output_height & 1)) {
        return NULL;
    }
    rescue_vse_scaler_t *scaler = calloc(1, sizeof(*scaler));
    if (scaler == NULL) return NULL;
    scaler->vnode = -1;
    scaler->input.buffer.fd[0] = -1;
    scaler->input_width = input_width;
    scaler->input_height = input_height;
    scaler->output_width = output_width;
    scaler->output_height = output_height;

    if (hb_mem_module_open() != 0) goto fail;
    scaler->memory_open = 1;
    const int64_t input_flags =
        HB_MEM_USAGE_MAP_INITIALIZED |
        HB_MEM_USAGE_PRIV_HEAP_2_RESERVED |
        HB_MEM_USAGE_CPU_READ_OFTEN |
        HB_MEM_USAGE_CPU_WRITE_OFTEN |
        HB_MEM_USAGE_CACHED |
        HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF;
    if (hb_mem_alloc_graph_buf(input_width, input_height, MEM_PIX_FMT_NV12,
                               input_flags, input_width, input_height,
                               &scaler->input.buffer) != 0) goto fail;

    if (hbn_vnode_open(HB_VSE, 0, AUTO_ALLOC_ID, &scaler->vnode) != 0) goto fail;
    scaler->vnode_open = 1;
    vse_attr_t node_attr = {0};
    if (hbn_vnode_set_attr(scaler->vnode, &node_attr) != 0) goto fail;

    vse_ichn_attr_t input_attr = {0};
    input_attr.width = (uint32_t)input_width;
    input_attr.height = (uint32_t)input_height;
    input_attr.fmt = FRM_FMT_NV12;
    input_attr.bit_width = 8;
    if (hbn_vnode_set_ichn_attr(scaler->vnode, 0, &input_attr) != 0) goto fail;

    vse_ochn_attr_t output_attr = {0};
    output_attr.chn_en = CAM_TRUE;
    output_attr.roi.x = 0;
    output_attr.roi.y = 0;
    output_attr.roi.w = (uint32_t)input_width;
    output_attr.roi.h = (uint32_t)input_height;
    output_attr.target_w = (uint32_t)output_width;
    output_attr.target_h = (uint32_t)output_height;
    output_attr.fmt = FRM_FMT_NV12;
    output_attr.bit_width = 8;
    if (hbn_vnode_set_ochn_attr(scaler->vnode, 0, &output_attr) != 0) goto fail;

    hbn_buf_alloc_attr_t alloc_attr = {0};
    alloc_attr.buffers_num = 3;
    alloc_attr.is_contig = 1;
    alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN | HB_MEM_USAGE_CACHED;
    if (hbn_vnode_set_ochn_buf_attr(scaler->vnode, 0, &alloc_attr) != 0) goto fail;
    if (hbn_vnode_start(scaler->vnode) != 0) goto fail;
    scaler->started = 1;
    return scaler;

fail:
    destroy_scaler(scaler);
    free(scaler);
    return NULL;
}

int rescue_vse_scale(void *handle, const uint8_t *source, uint32_t source_size,
                     uint8_t *destination, uint32_t destination_size,
                     int timeout_ms)
{
    rescue_vse_scaler_t *scaler = handle;
    if (scaler == NULL || source == NULL || destination == NULL) return -1;
    const uint32_t source_required =
        (uint32_t)(scaler->input_width * scaler->input_height * 3 / 2);
    const uint32_t destination_required =
        (uint32_t)(scaler->output_width * scaler->output_height * 3 / 2);
    if (source_size < source_required || destination_size < destination_required) return -2;

    hb_mem_graphic_buf_t *input = &scaler->input.buffer;
    if (input->virt_addr[0] == NULL || input->virt_addr[1] == NULL ||
        input->stride < scaler->input_width) return -3;
    for (int row = 0; row < scaler->input_height; ++row) {
        memcpy(input->virt_addr[0] + row * input->stride,
               source + row * scaler->input_width, scaler->input_width);
    }
    const uint8_t *source_uv = source + scaler->input_width * scaler->input_height;
    for (int row = 0; row < scaler->input_height / 2; ++row) {
        memcpy(input->virt_addr[1] + row * input->stride,
               source_uv + row * scaler->input_width, scaler->input_width);
    }
    if (hb_mem_flush_buf(input->fd[0], 0, input->size[0] + input->size[1]) != 0) return -4;
    scaler->input.info.frame_id = scaler->frame_id++;
    gettimeofday(&scaler->input.info.tv, NULL);
    if (hbn_vnode_sendframe(scaler->vnode, 0, &scaler->input) != 0) return -5;

    hbn_vnode_image_t output = {0};
    if (hbn_vnode_getframe(scaler->vnode, 0, (uint32_t)timeout_ms, &output) != 0) return -6;
    int result = 0;
    hb_mem_graphic_buf_t *buffer = &output.buffer;
    if (buffer->virt_addr[0] == NULL || buffer->virt_addr[1] == NULL ||
        buffer->stride < scaler->output_width) {
        result = -7;
        goto release;
    }
    if (hb_mem_invalidate_buf(buffer->fd[0], 0, buffer->size[0] + buffer->size[1]) != 0) {
        result = -8;
        goto release;
    }
    for (int row = 0; row < scaler->output_height; ++row) {
        memcpy(destination + row * scaler->output_width,
               buffer->virt_addr[0] + row * buffer->stride, scaler->output_width);
    }
    uint8_t *destination_uv = destination + scaler->output_width * scaler->output_height;
    for (int row = 0; row < scaler->output_height / 2; ++row) {
        memcpy(destination_uv + row * scaler->output_width,
               buffer->virt_addr[1] + row * buffer->stride, scaler->output_width);
    }

release:
    if (hbn_vnode_releaseframe(scaler->vnode, 0, &output) != 0 && result == 0) result = -9;
    return result == 0 ? (int)destination_required : result;
}

void rescue_vse_destroy(void *handle)
{
    rescue_vse_scaler_t *scaler = handle;
    if (scaler == NULL) return;
    destroy_scaler(scaler);
    free(scaler);
}
