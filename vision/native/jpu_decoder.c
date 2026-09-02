#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <hb_media_codec.h>

typedef struct {
    media_codec_context_t codec;
    int width;
    int height;
    int started;
} rdk_jpu_decoder_t;

void *rdk_jpu_create(int width, int height)
{
    if (width <= 0 || height <= 0 || (width & 1) || (height & 1)) {
        return NULL;
    }
    rdk_jpu_decoder_t *decoder = calloc(1, sizeof(*decoder));
    if (decoder == NULL) {
        return NULL;
    }

    decoder->width = width;
    decoder->height = height;
    decoder->codec.encoder = 0;
    decoder->codec.codec_id = MEDIA_CODEC_ID_JPEG;
    mc_video_codec_dec_params_t *params = &decoder->codec.video_dec_params;
    params->feed_mode = MC_FEEDING_MODE_FRAME_SIZE;
    params->pix_fmt = MC_PIXEL_FORMAT_NV12;
    params->bitstream_buf_size = (width * height * 2 + 0xFFF) & ~0xFFF;
    params->bitstream_buf_count = 3;
    params->frame_buf_count = 3;
    params->jpeg_dec_config.frame_crop_enable = 0;
    params->jpeg_dec_config.rot_degree = MC_CCW_0;
    params->jpeg_dec_config.mir_direction = MC_DIRECTION_NONE;

    if (hb_mm_mc_initialize(&decoder->codec) != 0) {
        free(decoder);
        return NULL;
    }
    if (hb_mm_mc_configure(&decoder->codec) != 0) {
        hb_mm_mc_release(&decoder->codec);
        free(decoder);
        return NULL;
    }
    mc_av_codec_startup_params_t startup = {0};
    if (hb_mm_mc_start(&decoder->codec, &startup) != 0) {
        hb_mm_mc_release(&decoder->codec);
        free(decoder);
        return NULL;
    }
    decoder->started = 1;
    return decoder;
}

int rdk_jpu_decode(void *handle, const uint8_t *jpeg, uint32_t jpeg_size,
                   uint8_t *nv12, uint32_t nv12_capacity)
{
    rdk_jpu_decoder_t *decoder = handle;
    if (decoder == NULL || jpeg == NULL || jpeg_size == 0 || nv12 == NULL) {
        return -1;
    }
    const uint32_t required = (uint32_t)(decoder->width * decoder->height * 3 / 2);
    if (nv12_capacity < required) {
        return -2;
    }

    media_codec_buffer_t input = {0};
    input.type = MC_VIDEO_STREAM_BUFFER;
    int ret = hb_mm_mc_dequeue_input_buffer(&decoder->codec, &input, 20);
    if (ret != 0) {
        return -10;
    }
    if (input.vstream_buf.size < jpeg_size) {
        input.vstream_buf.size = 0;
        input.vstream_buf.stream_end = 0;
        hb_mm_mc_queue_input_buffer(&decoder->codec, &input, 0);
        return -11;
    }
    memcpy(input.vstream_buf.vir_ptr, jpeg, jpeg_size);
    input.vstream_buf.size = jpeg_size;
    input.vstream_buf.stream_end = 0;
    ret = hb_mm_mc_queue_input_buffer(&decoder->codec, &input, 20);
    if (ret != 0) {
        return -12;
    }

    media_codec_buffer_t output = {0};
    media_codec_output_buffer_info_t info = {0};
    /* The first frame can take longer while JPU firmware/buffers warm up.
     * This is a maximum wait, not an added delay: normal frames return as
     * soon as hardware completes. */
    ret = hb_mm_mc_dequeue_output_buffer(&decoder->codec, &output, &info, 200);
    if (ret != 0) {
        hb_mm_mc_flush(&decoder->codec);
        return -20;
    }
    if (output.type != MC_VIDEO_FRAME_BUFFER ||
        info.jpeg_frame_info.decode_result == 0 ||
        output.vframe_buf.vir_ptr[0] == NULL ||
        output.vframe_buf.vir_ptr[1] == NULL) {
        hb_mm_mc_queue_output_buffer(&decoder->codec, &output, 0);
        return -21;
    }

    const int source_width = output.vframe_buf.width > 0 ? output.vframe_buf.width : output.vframe_buf.stride;
    const int source_stride = output.vframe_buf.stride > 0 ? output.vframe_buf.stride : source_width;
    if (source_width < decoder->width || source_stride < decoder->width) {
        hb_mm_mc_queue_output_buffer(&decoder->codec, &output, 0);
        return -22;
    }
    for (int row = 0; row < decoder->height; ++row) {
        memcpy(nv12 + row * decoder->width,
               output.vframe_buf.vir_ptr[0] + row * source_stride,
               decoder->width);
    }
    uint8_t *uv = nv12 + decoder->width * decoder->height;
    for (int row = 0; row < decoder->height / 2; ++row) {
        memcpy(uv + row * decoder->width,
               output.vframe_buf.vir_ptr[1] + row * source_stride,
               decoder->width);
    }
    ret = hb_mm_mc_queue_output_buffer(&decoder->codec, &output, 0);
    return ret == 0 ? (int)required : -23;
}

void rdk_jpu_destroy(void *handle)
{
    rdk_jpu_decoder_t *decoder = handle;
    if (decoder == NULL) {
        return;
    }
    if (decoder->started) {
        hb_mm_mc_stop(&decoder->codec);
    }
    hb_mm_mc_release(&decoder->codec);
    free(decoder);
}
