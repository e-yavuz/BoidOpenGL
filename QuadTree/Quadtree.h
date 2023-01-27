#include <vector>

#ifndef QUADTREE_H
#define QUADTREE_H

template<typename T>
struct point_bucket
{
    point_bucket(float x, float y, float x_length, float y_length) : x{x}, y{y}, x_length{x_length}, y_length{y_length} {}

    point_bucket(float x, float y, float x_length, float y_length, std::vector<T*>& bucket) : x{x}, y{y}, x_length{x_length}, y_length{y_length}, bucket{bucket} {}

    point_bucket(float x, float y, float x_length, float y_length, std::vector<T>& bucket) : x{x}, y{y}, x_length{x_length}, y_length{y_length} 
    {
        this->bucket.reserve(bucket.size());
        for(auto& elm: bucket)
            this->bucket.push_back(&elm);
    }

    float x;
    float y;
    float x_length;
    float y_length;
    std::vector<T*> bucket;
};

template<typename T>
void base_split(point_bucket<T>& base, size_t max_size, std::vector<point_bucket<T> >& tree, float min_resolution);

#endif