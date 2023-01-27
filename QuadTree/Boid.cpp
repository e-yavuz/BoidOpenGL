#include "Boid.h"
#include <cstdlib>
#include <random>
#include <iostream>

void base_split(point_bucket<Boid>& base, size_t max_size, std::vector<point_bucket<Boid> >& tree, float min_resolution);

int main()
{
    Flock flock;
    for(size_t i = 0; i < 100; i++)
        flock.boids.emplace_back(std::vector<float>{(static_cast<float>(std::rand())/(static_cast<float>(RAND_MAX)/2))-1, (static_cast<float>(std::rand())/(static_cast<float>(RAND_MAX)/2))-1});

    std::vector<point_bucket<Boid> > tree;
    point_bucket<Boid> base(0, 0, 2, 2, flock.boids);
    base_split(base, 8, tree, 0.001f);
    for(auto& elm: tree)
    {
        flock.Update(elm.bucket);
    }
}

