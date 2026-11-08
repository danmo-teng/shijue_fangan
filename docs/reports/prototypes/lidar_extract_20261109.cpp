#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numeric>
#include <vector>

extern "C" int lidar_best_hypothesis(const double *,std::size_t,const std::int64_t *,std::size_t,double,double,std::uint8_t *);
struct Point {double x,y;};
struct Line {std::int64_t count;double normal[2],span,points[64];};
// Experiment only: call this machine's SAME NumPy LAPACK implementation.
using Svd=std::int64_t (*)(int,char,std::int64_t,std::int64_t,double *,std::int64_t,double *,double *,std::int64_t,double *,std::int64_t);
using Pairs=int (*)(std::size_t,std::int64_t *);
using Order=int (*)(const double *,std::size_t,std::int64_t *);

static bool tls(const std::vector<Point> &points,Point &center,double *vt,Svd svd,std::vector<double> &a,std::vector<double> &u) {
    center={0,0};for(const auto &p:points) {center.x+=p.x;center.y+=p.y;}
    center.x/=points.size();center.y/=points.size();
    const auto n=points.size();a.resize(2*n);u.resize(2*n);
    for(std::size_t i=0;i<n;++i) {a[i]=points[i].x-center.x;a[n+i]=points[i].y-center.y;}
    double singular[2];
    return svd(102,'S',n,2,a.data(),n,singular,u.data(),n,vt,2)==0;
}

static double quantile(const std::vector<double> &projection,const std::vector<std::int64_t> &order,
                       std::size_t begin,std::size_t end,double q) {
    const double rank=(end-begin-1)*q;const auto lower=static_cast<std::size_t>(rank);
    const double f=rank-lower,a=projection[order[begin+lower]],b=projection[order[std::min(begin+lower+1,end-1)]],d=b-a;
    return f<.5?a+d*f:b-d*(1-f);
}

extern "C" int lidar_extract_prototype(const double *xy,std::size_t n,double threshold,std::size_t min_points,double min_span,
    Pairs pairs,Order sorted_order,Svd svd,Line *out,std::size_t capacity,std::size_t *count) {
    if(!xy||!pairs||!sorted_order||!svd||!out||!count||min_points<2)return -1;
    *count=0;std::vector<Point> remaining;remaining.reserve(n);
    for(std::size_t i=0;i<n;++i)if(std::isfinite(xy[2*i])&&std::isfinite(xy[2*i+1]))remaining.push_back({xy[2*i],xy[2*i+1]});
    std::vector<Point> fit,next;fit.reserve(n);next.reserve(n);
    std::vector<double> a,u,projection;std::vector<std::int64_t> order;std::vector<std::uint8_t> mask(n);
    for(int round=0;round<8;++round) {
        n=remaining.size();if(n<min_points)break;
        std::int64_t hypotheses[320];if(!pairs(n,hypotheses))return -2;
        if(lidar_best_hypothesis(reinterpret_cast<const double *>(remaining.data()),n,hypotheses,160,threshold,.15,mask.data())<0)return -3;
        fit.clear();for(std::size_t i=0;i<n;++i)if(mask[i])fit.push_back(remaining[i]);
        if(fit.size()<min_points)break;
        Point center;double vt[4];
        if(!tls(fit,center,vt,svd,a,u))return -4;
        const double nx=vt[1],ny=vt[3];fit.clear();next.clear();
        for(const auto &p:remaining) {
            if(std::abs((p.x-center.x)*nx+(p.y-center.y)*ny)<threshold)fit.push_back(p);
            else next.push_back(p);
        }
        remaining.swap(next);if(fit.size()<min_points)continue;
        if(!tls(fit,center,vt,svd,a,u))return -4;
        const double tx=vt[0],ty=vt[2];projection.resize(fit.size());order.resize(fit.size());
        for(std::size_t i=0;i<fit.size();++i)projection[i]=(fit[i].x-center.x)*tx+(fit[i].y-center.y)*ty;
        // NumPy's default quicksort has a specific order for equal projections.
        // Keep that original ordering: std::sort can select different support
        // points, changing a wall posterior by millimetres on symmetric input.
        if(!sorted_order(projection.data(),projection.size(),order.data()))return -6;
        for(std::size_t begin=0,end=1;end<=order.size();++end) {
            if(end<order.size()&&projection[order[end]]-projection[order[end-1]]<=.15)continue;
            const auto size=end-begin;
            if(size>=min_points) {
                const double lo=quantile(projection,order,begin,end,.05),hi=quantile(projection,order,begin,end,.95);
                if(hi-lo>=min_span) {
                    if(*count>=capacity)return -5;
                    Line &line=out[(*count)++];line.count=std::min(std::size_t(32),size);
                    line.normal[0]=vt[1];line.normal[1]=vt[3];line.span=hi-lo;
                    const double step=double(size-1)/double(line.count-1);
                    for(std::size_t j=0;j<std::size_t(line.count);++j) {
                        const auto index=begin+(j+1==std::size_t(line.count)?size-1:static_cast<std::size_t>(j*step));
                        const auto p=fit[order[index]];line.points[2*j]=p.x;line.points[2*j+1]=p.y;
                    }
                }
            }
            begin=end;
        }
    }
    return 0;
}
