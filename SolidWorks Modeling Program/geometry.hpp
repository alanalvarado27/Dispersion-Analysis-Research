#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>
namespace sketch {
struct Point { double x, y; };
using Loop = std::vector<Point>;
using Loops = std::vector<Loop>;
inline double distance(Point a, Point b) { return std::hypot(a.x-b.x,a.y-b.y); }
inline double cross(Point a, Point b, Point c) {
    return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
}
inline bool onSegment(Point a, Point b, Point p) {
    return std::abs(cross(a,b,p)) < 1e-8 &&
        p.x >= std::min(a.x,b.x)-1e-8 && p.x <= std::max(a.x,b.x)+1e-8 &&
        p.y >= std::min(a.y,b.y)-1e-8 && p.y <= std::max(a.y,b.y)+1e-8;
}
inline bool intersects(Point a, Point b, Point c, Point d) {
    double u=cross(a,b,c), v=cross(a,b,d), w=cross(c,d,a), z=cross(c,d,b);
    return (((u>0 && v<0)||(u<0 && v>0)) && ((w>0 && z<0)||(w<0 && z>0))) ||
        onSegment(a,b,c)||onSegment(a,b,d)||onSegment(c,d,a)||onSegment(c,d,b);
}
inline Point nearestOnEdge(Point p, Point a, Point b) {
    double dx=b.x-a.x, dy=b.y-a.y, len=dx*dx+dy*dy;
    double t=len>0 ? std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/len,0.0,1.0):0;
    return {a.x+t*dx,a.y+t*dy};
}
inline void validate(const Loops& loops, int width, int height) {
    if(loops.empty() || loops.size()>32) throw std::runtime_error("Need 1 to 32 closed contours.");
    size_t total=0;
    for(const auto& p:loops) {
        total+=p.size();
        if(p.size()<3 || total>2000) throw std::runtime_error("Need at least 3 points per contour; maximum 2000 total.");
        double area=0;
        for(size_t i=0;i<p.size();++i) {
            Point a=p[i], b=p[(i+1)%p.size()], c=p[(i+2)%p.size()];
            if(!std::isfinite(a.x)||!std::isfinite(a.y)||a.x<0||a.y<0||a.x>width||a.y>height)
                throw std::runtime_error("A vertex is outside the image or is not finite.");
            if(distance(a,b)<0.25) throw std::runtime_error("Duplicate points or an edge shorter than 0.25 pixel.");
            if(std::abs(cross(a,b,c))<1e-8 &&
                ((b.x-a.x)*(c.x-b.x)+(b.y-a.y)*(c.y-b.y))<0)
                throw std::runtime_error("Adjacent edges double back and overlap.");
            area+=a.x*b.y-b.x*a.y;
            for(size_t j=i+1;j<p.size();++j) {
                if(j==i+1 || (i==0 && j==p.size()-1)) continue;
                if(intersects(a,b,p[j],p[(j+1)%p.size()]))
                    throw std::runtime_error("Contour crosses or touches itself. Adjust the vertices.");
            }
        }
        if(std::abs(area)<2) throw std::runtime_error("Contour area is too small.");
    }
    for(size_t l=0;l<loops.size();++l) for(size_t m=l+1;m<loops.size();++m)
        for(size_t i=0;i<loops[l].size();++i) for(size_t j=0;j<loops[m].size();++j)
            if(intersects(loops[l][i],loops[l][(i+1)%loops[l].size()],
                          loops[m][j],loops[m][(j+1)%loops[m].size()]))
                throw std::runtime_error("Separate contours touch or cross. Adjust the vertices.");
}
inline Point toMeters(Point p, Point origin, double mmPerPixel) {
    return {(p.x-origin.x)*mmPerPixel/1000.0,(origin.y-p.y)*mmPerPixel/1000.0};
}
}
