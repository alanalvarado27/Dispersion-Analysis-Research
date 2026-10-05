#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace sketch {
constexpr double pi=3.14159265358979323846;
struct Point { double x=0,y=0; };
inline Point operator+(Point a,Point b){return {a.x+b.x,a.y+b.y};}
inline Point operator-(Point a,Point b){return {a.x-b.x,a.y-b.y};}
inline Point operator*(Point a,double s){return {a.x*s,a.y*s};}
inline double dot(Point a,Point b){return a.x*b.x+a.y*b.y;}
inline double cross(Point a,Point b){return a.x*b.y-a.y*b.x;}
inline double distance(Point a,Point b){return std::hypot(a.x-b.x,a.y-b.y);}
inline Point nearestOnEdge(Point p,Point a,Point b){auto d=b-a;double v=dot(d,d);return a+d*(v?std::clamp(dot(p-a,d)/v,0.0,1.0):0);}
inline Point toMeters(Point p,Point o,double scale){return {(p.x-o.x)*scale/1000,(o.y-p.y)*scale/1000};}
enum class Kind {Line,Arc,Circle};
// Line: a=start, b=end. Arc: a=start, b=end, c=point on arc.
// Circle: a=center, b=point on circumference. All are shared node indices.
struct Entity {Kind kind=Kind::Line;int a=-1,b=-1,c=-1;};
struct Contour {std::string name;std::vector<Entity> entities;std::string role;};
struct Document {std::vector<Point> points;std::vector<std::string> ids;std::vector<Contour> contours;};
struct Arc {Point center;double radius,start,sweep;};
inline double positiveAngle(double a){a=std::fmod(a,2*pi);return a<0?a+2*pi:a;}
inline Arc arcThrough(Point a,Point b,Point m){
    Point u=b-a,v=m-a; double d=2*cross(u,v);
    if(distance(a,b)<0.05 || distance(a,m)<0.05 || distance(b,m)<0.05 ||
       std::abs(d)<1e-8*std::max(1.0,dot(u,u)+dot(v,v)))
        throw std::runtime_error("Arc points are coincident or collinear; drag the middle handle off the chord.");
    Point offset{(dot(u,u)*v.y-dot(v,v)*u.y)/d,(u.x*dot(v,v)-v.x*dot(u,u))/d};
    Point center=a+offset; double t=std::atan2(a.y-center.y,a.x-center.x);
    double end=positiveAngle(std::atan2(b.y-center.y,b.x-center.x)-t);
    double mid=positiveAngle(std::atan2(m.y-center.y,m.x-center.x)-t);
    return {center,distance(a,center),t,mid<end?end:end-2*pi};
}
inline std::vector<Point> sample(const Document& d,const Entity& e){
    Point a=d.points.at(e.a),b=d.points.at(e.b);
    if(e.kind==Kind::Line)return {a,b};
    Arc ar;
    if(e.kind==Kind::Circle)ar={a,distance(a,b),0,2*pi};
    else ar=arcThrough(a,b,d.points.at(e.c));
    // Rendering / screening tolerance 0.03 image pixels, native curves are preserved for CAD.
    double step=2*std::acos(std::clamp(1-0.03/std::max(ar.radius,0.03),-1.0,1.0));
    int n=std::clamp(int(std::ceil(std::abs(ar.sweep)/std::max(step,0.001))),8,4096);
    std::vector<Point> p; for(int i=0;i<=n;++i){double t=ar.start+ar.sweep*i/n;p.push_back(ar.center+Point{std::cos(t),std::sin(t)}*ar.radius);}
    if(e.kind!=Kind::Circle){p.front()=a;p.back()=b;}else p.back()=p.front();
    return p;
}
inline std::vector<Point> sampleContour(const Document& d,const Contour& c){
    std::vector<Point> out;
    for(const auto& e:c.entities){auto s=sample(d,e);out.insert(out.end(),s.begin(),s.end()-1);}
    return out;
}
inline bool segmentsMeet(Point a,Point b,Point c,Point d){
    constexpr double eps=1e-7;
    if(std::max(a.x,b.x)+eps<std::min(c.x,d.x)||std::max(c.x,d.x)+eps<std::min(a.x,b.x)||
       std::max(a.y,b.y)+eps<std::min(c.y,d.y)||std::max(c.y,d.y)+eps<std::min(a.y,b.y))return false;
    double p=cross(b-a,c-a),q=cross(b-a,d-a),r=cross(d-c,a-c),s=cross(d-c,b-c);
    return ((p>=-eps&&q<=eps)||(q>=-eps&&p<=eps))&&((r>=-eps&&s<=eps)||(s>=-eps&&r<=eps));
}
inline void validate(const Document& d,int w,int h,bool bounded=true){
    if(d.contours.empty()||d.contours.size()>128||d.points.empty()||d.points.size()>6000)
        throw std::runtime_error("Sketch needs 1..128 contours and 1..6000 points.");
    for(auto p:d.points)if(!std::isfinite(p.x)||!std::isfinite(p.y)||(bounded&&(p.x<0||p.x>w||p.y<0||p.y>h)))
        throw std::runtime_error("A control point is outside the image or nonfinite.");
    std::vector<std::vector<Point>> loops; size_t edges=0;
    for(const auto& c:d.contours){
        if(c.entities.empty())throw std::runtime_error("Empty contour: "+c.name);
        for(size_t i=0;i<c.entities.size();++i){
            const auto& e=c.entities[i];
            if(e.a<0||e.b<0||size_t(e.a)>=d.points.size()||size_t(e.b)>=d.points.size()||
               (e.kind==Kind::Arc&&(e.c<0||size_t(e.c)>=d.points.size())))throw std::runtime_error("Invalid point reference.");
            if(distance(d.points[e.a],d.points[e.b])<0.05)throw std::runtime_error("An edge or circle radius is under 0.05 pixels.");
            if(e.kind==Kind::Circle){if(c.entities.size()!=1)throw std::runtime_error("Each circle must have its own contour.");}
            else if(e.b!=c.entities[(i+1)%c.entities.size()].a)
                throw std::runtime_error("Open contour: "+c.name+". Consecutive entities must share endpoint IDs.");
        }
        auto p=sampleContour(d,c);double area=0;
        for(size_t i=0;i<p.size();++i){
            if(bounded&&(p[i].x<-0.01||p[i].y<-0.01||p[i].x>w+0.01||p[i].y>h+0.01))
                throw std::runtime_error("A curve leaves the image.");
            area+=cross(p[i],p[(i+1)%p.size()]);
        }
        if(p.size()<3||std::abs(area)<0.1)throw std::runtime_error("Degenerate contour: "+c.name);
        edges+=p.size();if(edges>30000)throw std::runtime_error("Too many sampled edges; simplify the sketch.");
        loops.push_back(p);
    }
    auto inside=[](Point p,const std::vector<Point>& poly){
        bool in=false;for(size_t i=0,j=poly.size()-1;i<poly.size();j=i++){
            Point a=poly[i],b=poly[j];
            if((a.y>p.y)!=(b.y>p.y))if(p.x<(b.x-a.x)*(p.y-a.y)/(b.y-a.y)+a.x)in=!in;
        }return in;
    };
    for(size_t i=0;i<loops.size();++i){
        const auto& role=d.contours[i].role;
        if(role.empty()||role=="auto")continue;
        if(role!="solid"&&role!="hole")throw std::runtime_error("Contour role must be solid, hole, or auto.");
        int depth=0;for(size_t j=0;j<loops.size();++j)if(i!=j&&inside(loops[i][0],loops[j]))++depth;
        if((role=="hole")!=(depth%2==1))throw std::runtime_error("Material role conflicts with contour nesting: "+d.contours[i].name);
    }
    for(size_t l=0;l<loops.size();++l)for(size_t k=l;k<loops.size();++k){
        auto& a=loops[l];auto& b=loops[k];
        for(size_t i=0;i<a.size();++i)for(size_t j=(k==l?i+1:0);j<b.size();++j){
            if(k==l&&(j==i+1||(i==0&&j==a.size()-1)))continue;
            if(segmentsMeet(a[i],a[(i+1)%a.size()],b[j],b[(j+1)%b.size()]))
                throw std::runtime_error("Contours cross, touch, or overlap: "+d.contours[l].name+" / "+d.contours[k].name);
        }
        // Adjacent edges may meet, but may not double back and overlap.
        if(k==l)for(size_t i=0;i<a.size();++i){Point u=a[i]-a[(i+a.size()-1)%a.size()],v=a[(i+1)%a.size()]-a[i];
            if(std::abs(cross(u,v))<1e-7&&dot(u,v)<0)throw std::runtime_error("Contour doubles back on itself.");}
    }
}
inline void movePoint(Document& d,int index,Point p){
    Point delta=p-d.points.at(index);
    for(const auto& c:d.contours)for(const auto& e:c.entities)
        if(e.kind==Kind::Circle&&e.a==index)d.points[e.b]=d.points[e.b]+delta;
    d.points[index]=p;
}
inline int addPoint(Document& d,Point p){
    int n=int(d.points.size());std::string id="edit_"+std::to_string(n);
    while(std::find(d.ids.begin(),d.ids.end(),id)!=d.ids.end())id+="_";
    d.points.push_back(p);d.ids.push_back(id);return n;
}
inline void splitLine(Document& d,size_t c,size_t e,Point p){
    auto& es=d.contours.at(c).entities;auto old=es.at(e);if(old.kind!=Kind::Line)return;
    if(distance(p,d.points[old.a])<0.1||distance(p,d.points[old.b])<0.1)return;
    int n=addPoint(d,p);es[e].b=n;es.insert(es.begin()+e+1,Entity{Kind::Line,n,old.b,-1});
}
inline bool removeLinePoint(Document& d,int n){
    int refs=0;for(const auto& c:d.contours)for(const auto& e:c.entities)
        refs+=(e.a==n)+(e.b==n)+(e.c==n);
    if(refs!=2)return false;
    for(auto& c:d.contours){auto& es=c.entities;if(es.size()<=3)continue;
        for(size_t i=0;i<es.size();++i){size_t j=(i+1)%es.size();
            if(es[i].kind==Kind::Line&&es[j].kind==Kind::Line&&es[i].b==n&&es[j].a==n){
                es[i].b=es[j].b;es.erase(es.begin()+j);
                d.points.erase(d.points.begin()+n);d.ids.erase(d.ids.begin()+n);
                for(auto& cc:d.contours)for(auto& e:cc.entities){if(e.a>n)--e.a;if(e.b>n)--e.b;if(e.c>n)--e.c;}
                return true;
            }
        }
    }return false;
}
}
