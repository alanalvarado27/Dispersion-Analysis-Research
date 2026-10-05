#include "geometry.hpp"
#include <cassert>
#include <iostream>
using namespace sketch;
bool rejects(const Loops& p) {try {validate(p,100,100);return false;} catch(const std::exception&) {return true;}}
int main() {
    Loop square={{0,0},{100,0},{100,100},{0,100}};
    validate({square},100,100);
    validate({{{0,0},{100,0},{100,40},{40,40},{40,100},{0,100}}},100,100);
    validate({square,{{20,20},{30,20},{30,30},{20,30}}},100,100);
    assert(rejects({{{0,0},{100,100},{0,100},{100,0}}})); // bow tie
    assert(rejects({{{0,0},{100,0},{50,0},{50,50},{0,50}}})); // backtracking
    assert(rejects({{{0,0},{0,0},{100,0},{0,100}}}));
    assert(rejects({{{0,0},{-1,10},{50,50}}}));
    assert(rejects({square,{{0,20},{30,20},{30,30},{0,30}}})); // touching hole
    assert(rejects({{{0,0},{10,0},{20,0}}}));
    assert(rejects({}));
    assert(rejects({{{0,0},{NAN,10},{50,50}}}));
    assert(rejects({{{0,0},{100,0},{100,100},{50,0},{0,100}}})); // nonadjacent touch
    auto p=toMeters({200,300},{100,100},0.5);
    assert(std::abs(p.x-0.05)<1e-12 && std::abs(p.y+0.1)<1e-12);
    auto q=nearestOnEdge({7,4},{0,0},{10,0});
    assert(q.x==7 && q.y==0);
    std::cout<<"Geometry checks passed: concavity, holes, crossings, touching, degeneracy, bounds, scale and Y flip.\n";
}
