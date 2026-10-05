#pragma once
#include "offline_geometry.hpp"
#include "json.hpp"
#include <map>
#include <set>
#include <sstream>

namespace sketch {
using Json=nlohmann::json;
struct Equation {std::string type,label;std::vector<int> p;double target=0;};
struct DimensionSystem {
    std::vector<Equation> equations;
    Json dimensions=Json::array(),relations=Json::array();
    std::map<std::string,double> values;
    double scale=1;
};
inline int reference(const Document& d,const std::string& id){
    auto it=std::find(d.ids.begin(),d.ids.end(),id);
    if(it==d.ids.end())throw std::runtime_error("Unknown dimension/constraint point: "+id);
    return int(it-d.ids.begin());
}
inline int arity(const std::string& t){
    if(t=="distance"||t=="horizontal_distance"||t=="vertical_distance"||t=="radius"||t=="diameter"||
       t=="horizontal"||t=="vertical")return 2;
    if(t=="midpoint_x"||t=="midpoint_y")return 3;
    if(t=="equal_length"||t=="parallel"||t=="perpendicular")return 4;
    throw std::runtime_error("Unsupported dimension/constraint type: "+t);
}
inline Equation readEquation(const Document& d,const Json& j){
    Equation e;e.type=j.at("type").get<std::string>();e.label=j.value("name",e.type);
    const auto& p=j.at("points");if(!p.is_array()||p.size()!=size_t(arity(e.type)))throw std::runtime_error("Wrong point count for "+e.type);
    for(const auto& id:p)e.p.push_back(reference(d,id.get<std::string>()));
    return e;
}
inline double measured(const Document& d,const Equation& e){
    auto a=d.points.at(e.p[0]),b=d.points.at(e.p[1]);
    if(e.type=="horizontal_distance")return std::abs(b.x-a.x);
    if(e.type=="vertical_distance")return std::abs(b.y-a.y);
    if(e.type=="diameter")return 2*distance(a,b);
    if(e.type=="distance"||e.type=="radius")return distance(a,b);
    if(e.type=="horizontal")return b.y-a.y;
    if(e.type=="vertical")return b.x-a.x;
    if(e.type=="midpoint_x")return a.x-(b.x+d.points.at(e.p[2]).x)/2;
    if(e.type=="midpoint_y")return a.y-(b.y+d.points.at(e.p[2]).y)/2;
    auto u=b-a,v=d.points.at(e.p[3])-d.points.at(e.p[2]);
    if(e.type=="equal_length")return std::hypot(u.x,u.y)-std::hypot(v.x,v.y);
    double length=std::max({std::hypot(u.x,u.y),std::hypot(v.x,v.y),1e-6});
    if(e.type=="parallel")return cross(u,v)/length;
    return dot(u,v)/length;
}
inline bool isDimension(const std::string& t){return t=="distance"||t=="horizontal_distance"||t=="vertical_distance"||t=="radius"||t=="diameter";}
inline DimensionSystem makeDimensions(const Document& d,const Json& source,const std::map<std::string,double>& values,double scale){
    DimensionSystem sys;sys.scale=scale;sys.values=values;
    sys.dimensions=source.value("dimensions",Json::array());sys.relations=source.value("constraints",Json::array());
    if(!sys.dimensions.is_array()||!sys.relations.is_array()||sys.dimensions.size()+sys.relations.size()>128)
        throw std::runtime_error("Use at most 128 dimension/constraint equations.");
    if(!std::isfinite(scale)||scale<=0)throw std::runtime_error("Invalid dimension scale.");
    for(auto& j:sys.dimensions){
        auto e=readEquation(d,j);if(!isDimension(e.type))throw std::runtime_error("Invalid dimension type.");
        auto it=values.find(j.at("name").get<std::string>());if(it==values.end())throw std::runtime_error("Missing dimension value.");
        if(!std::isfinite(it->second)||it->second<=0)throw std::runtime_error("Dimensions must be finite and positive.");
        e.target=it->second/scale;j["value_mm"]=it->second;sys.equations.push_back(e);
    }
    for(const auto& j:sys.relations){auto e=readEquation(d,j);if(isDimension(e.type))throw std::runtime_error("Put dimensional values in dimensions, not constraints.");sys.equations.push_back(e);}
    return sys;
}
inline double dimensionError(const Document& d,const DimensionSystem& sys){
    double error=0;for(const auto& e:sys.equations)error=std::max(error,std::abs(measured(d,e)-e.target));return error;
}
inline std::vector<double> linearSolve(std::vector<std::vector<double>> a,std::vector<double> b){
    int n=int(b.size());for(int k=0;k<n;++k){
        int pivot=k;for(int i=k+1;i<n;++i)if(std::abs(a[i][k])>std::abs(a[pivot][k]))pivot=i;
        if(std::abs(a[pivot][k])<1e-16)throw std::runtime_error("Singular dimension system.");
        std::swap(a[pivot],a[k]);std::swap(b[pivot],b[k]);
        for(int i=k+1;i<n;++i){double f=a[i][k]/a[k][k];for(int j=k;j<n;++j)a[i][j]-=f*a[k][j];b[i]-=f*b[k];}
    }
    std::vector<double>x(n);for(int i=n-1;i>=0;--i){double v=b[i];for(int j=i+1;j<n;++j)v-=a[i][j]*x[j];x[i]=v/a[i][i];}return x;
}
inline void solveDimensions(Document& document,const DimensionSystem& sys,int pinned=-1){
    if(sys.equations.empty())return;
    Document d=document;Point pin=pinned>=0?d.points.at(pinned):Point{};
    std::set<int> active;for(const auto& e:sys.equations)for(int p:e.p)active.insert(p);
    if(pinned>=0)active.insert(pinned);
    std::vector<int> nodes(active.begin(),active.end());
    auto residual=[&](const Document& v){
        std::vector<double> r;for(const auto& e:sys.equations)r.push_back(measured(v,e)-e.target);
        if(pinned>=0){r.push_back(v.points[pinned].x-pin.x);r.push_back(v.points[pinned].y-pin.y);}return r;
    };
    auto cost=[](const std::vector<double>& r){double c=0;for(double v:r)c+=v*v;return c;};
    // Damped minimum-displacement projection. Underconstrained coordinates keep
    // the image-derived arrangement as far as the requested equations allow.
    for(int iteration=0;iteration<100;++iteration){
        auto r=residual(d);double maxError=0;for(double v:r)maxError=std::max(maxError,std::abs(v));
        if(maxError*sys.scale<1e-6){document=std::move(d);return;}
        int m=int(r.size()),n=int(nodes.size()*2);
        std::vector<std::vector<double>> J(m,std::vector<double>(n));
        for(int k=0;k<n;++k){
            double& x=(k%2?d.points[nodes[k/2]].y:d.points[nodes[k/2]].x);
            double saved=x,h=1e-5*std::max(1.0,std::abs(x)/100);
            x=saved+h;auto plus=residual(d);x=saved-h;auto minus=residual(d);x=saved;
            for(int i=0;i<m;++i)J[i][k]=(plus[i]-minus[i])/(2*h);
        }
        std::vector<std::vector<double>> A(m,std::vector<double>(m));
        for(int i=0;i<m;++i)for(int j=0;j<m;++j){for(int k=0;k<n;++k)A[i][j]+=J[i][k]*J[j][k];if(i==j)A[i][j]+=1e-8;}
        auto lambda=linearSolve(A,r);std::vector<double> step(n);
        for(int k=0;k<n;++k)for(int i=0;i<m;++i)step[k]-=J[i][k]*lambda[i];
        bool advanced=false;double initialCost=cost(r);
        for(double f=1;f>=1.0/1024;f*=0.5){
            Document candidate=d;for(int k=0;k<n;++k){double& x=k%2?candidate.points[nodes[k/2]].y:candidate.points[nodes[k/2]].x;x+=f*step[k];}
            double c=cost(residual(candidate));if(std::isfinite(c)&&c<initialCost){d=std::move(candidate);advanced=true;break;}
        }
        if(!advanced)break;
    }
    throw std::runtime_error("Dimension constraints cannot be satisfied. Check conflicting values/relations or undo the drag. No partial solution was committed.");
}
}
