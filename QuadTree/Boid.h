#ifndef BOID_H
#define BOID_H

#include "Quadtree.h"
#include <stdexcept>
#include <math.h>

typedef std::vector<size_t> size_t_vector;

class Flock;
class Boid;

template<typename T>
void CapVector(std::vector<T>& vector, T max_magnitude_squared, T min_magnitude_squared);
template<typename T>
std::vector<T> NormalizeVectorCopy(std::vector<T> vector, T magnitude);
template<typename T>
void NormalizeVectorInPlace(std::vector<T>& vector, T magnitude);
inline float Q_rsqrt(float number);

class Boid
{
public:
    std::vector<float> location;
    std::vector<float> velocity;
    std::vector<float> acceleration;
    Boid(std::vector<float> location, std::vector<float> velocity) : location(location), velocity(velocity), acceleration(location.size(), 0.f)
    {
        if(location.size()!= velocity.size()) throw "Incorrect dimensionality between location and velocity";
    }

    Boid(std::vector<float> location) : location(location), velocity(location.size(), 0.f), acceleration(location.size(), 0.f) {};

    void EmptyAcceleration()
    {
        for(size_t i = 0; i<velocity.size(); i++)
        {
            velocity[i] += acceleration[i];
            acceleration[i] = 0.f;
        }
    } 
};

class Flock
{
public:
    friend class Boid;
    std::vector<Boid> boids;

    Flock() {};

    Flock(float max_dist, float max_acceleration_magnitude) :
    max_dist(max_dist),
    max_acceleration_magnitude(max_acceleration_magnitude) 
    {};

    void Update(std::vector<Boid*>& miniFlock)
    {
        for(Boid* primary_elm: miniFlock)
        {
            CapVector(primary_elm->velocity, max_velocity_magnitude, max_velocity_magnitude/8);
            for(size_t i = 0; i < primary_elm->location.size(); i++)
                primary_elm->location[i]+=primary_elm->velocity[i];
        }
        
        if(miniFlock.size() < 2) return;

        for(Boid* primary_elm: miniFlock)
        {
            std::vector<float> average_velocity(primary_elm->location.size(), 0);
            std::vector<float> average_location(primary_elm->location.size(), 0);
            size_t valid_boid_count = 0;

            for(Boid* secondary_elm: miniFlock)
            {
                if(primary_elm == secondary_elm) continue;
                float dist = SquaredDistance<float>(primary_elm->location, secondary_elm->location);
                if(dist > max_dist) continue;
                valid_boid_count++;
                Seperation(primary_elm, secondary_elm, dist);
                
                size_t ind = 0;
                for(auto scalar: secondary_elm->location)
                    average_location[ind++]+=scalar;

                ind = 0;
                for(auto scalar: secondary_elm->velocity)
                    average_velocity[ind++]+=scalar;
            }
            if(!valid_boid_count) continue;
            
            CapVector(primary_elm->acceleration, max_acceleration_magnitude,max_acceleration_magnitude);
            for(auto& elm: average_location)
                elm/=valid_boid_count;

            for(auto& elm: average_velocity)
                elm/=valid_boid_count;

            Alignment(primary_elm, average_velocity);
            Cohesion(primary_elm, average_location);
        }

        for(Boid* primary_elm: miniFlock)
        {
            CapVector<float>(primary_elm->acceleration, max_acceleration_magnitude, max_acceleration_magnitude);
            primary_elm->EmptyAcceleration();
        }
    }

    void Mirror()
    {
        for(auto& boid: boids)
        {
            for(auto& val: boid.location)
            {
                if(val > 1.f) val-=2.f;
                else if(val < -1.f) val+=2.f;
            }
        }
    }

private:
    template<typename T>
    T SquaredDistance(std::vector<T>& A, std::vector<T>& B)
    {
        if(A.size() != B.size()) throw std::runtime_error("Unequal Dimensions between two Boids!");

        T retval = 0;
        for(size_t i = 0; i < A.size(); i++)
            retval += (A[i]-B[i])*(A[i]-B[i]);

        return retval;
    }

    void Seperation(Boid* const a, Boid* const b, float distance)
    {
        if(a==b || distance == 0.f) return;
        for(size_t i = 0; i < a->location.size(); i++)
            a->acceleration[i] += (a->location[i]-b->location[i])/(distance*distance);
    }

    void Alignment(Boid* const target, std::vector<float>& averageVelocity)
    {
        std::vector<float> retval(averageVelocity.size(), 0.f);
        size_t ind = 0;
        for(auto& elm: retval)
            elm+=(averageVelocity[ind]-target->velocity[ind]), ind++;
        CapVector(retval, max_acceleration_magnitude, max_acceleration_magnitude);
        for(size_t i = 0; i < target->acceleration.size(); i++)
            target->acceleration[i]+=retval[i];
    }

    void Cohesion(Boid* const target, std::vector<float>& averageLocation)
    {
        std::vector<float> retval(averageLocation.size(), 0.f);
        size_t ind = 0;
        for(auto& elm: retval)
            elm += (averageLocation[ind]-target->location[ind]-target->velocity[ind]), ind++;
        CapVector(retval, max_acceleration_magnitude, max_acceleration_magnitude);
        for(size_t i = 0; i < target->acceleration.size(); i++)
            target->acceleration[i]+=retval[i];
    }

private:
    float max_dist = 0.04f; // Max squared distance for a Boid to be in the flock
    float max_acceleration_magnitude = 0.0005f;
    float max_velocity_magnitude = 0.01f;

};

inline void base_split(point_bucket<Boid>& base, size_t max_size, std::vector<point_bucket<Boid> >& tree, size_t numberOfSeperations)
{
    //Empty starting buck et
    if(base.bucket.size() <= max_size || numberOfSeperations == 0 ) 
    {
        if(tree.empty()) tree.push_back(base);
        return;
    }
        

    base.x_length/=2;
    base.y_length/=2;

    point_bucket<Boid> NW(base.x - base.x_length/2, base.y + base.y_length/2, base.x_length, base.y_length);
    point_bucket<Boid> SW(base.x - base.x_length/2, base.y - base.y_length/2, base.x_length, base.y_length);
    point_bucket<Boid> SE(base.x + base.x_length/2, base.y - base.y_length/2, base.x_length, base.y_length);

    std::vector<Boid*> base_extra_bucket;
    for(auto iter = base.bucket.rbegin(); iter < base.bucket.rend(); iter++)
    {
        auto& elm = (*iter)->location;

        switch (static_cast<char>(elm[0] < base.x) + 2*static_cast<char>((elm[1] < base.y))) {
        case 0:
            base_extra_bucket.push_back(*iter);
            break;
        case 1:
            NW.bucket.push_back(*iter);
            break;
        case 2:
            SE.bucket.push_back(*iter);
            break;
        case 3:
            SW.bucket.push_back(*iter);
            break;
        }

        base.bucket.pop_back();
    }

    for(auto iter = base_extra_bucket.rbegin(); iter < base_extra_bucket.rend(); iter++)
        base.bucket.push_back(*iter);

    base.x += base.x_length/2;
    base.y += base.y_length/2;
    numberOfSeperations--;

    if(!base.bucket.empty()) 
        base.bucket.size() > max_size ? base_split(base, max_size, tree, numberOfSeperations):tree.push_back(base);
    if(!NW.bucket.empty()) 
        NW.bucket.size() > max_size ? base_split(NW, max_size, tree, numberOfSeperations):tree.push_back(NW);
    if(!SW.bucket.empty()) 
        SW.bucket.size() > max_size ? base_split(SW, max_size, tree, numberOfSeperations):tree.push_back(SW);
    if(!SE.bucket.empty()) 
        SE.bucket.size() > max_size ? base_split(SE, max_size, tree, numberOfSeperations):tree.push_back(SE);
}

template<typename T>
void CapVector(std::vector<T>& vector, T max_magnitude, T min_magnitude)
{
    T vector_scaled_magnitude, vector_magnitude_squared = 0;

    for(T& elm: vector) vector_magnitude_squared+=(elm*elm);

    if(vector_magnitude_squared==0) return;

    if(vector_magnitude_squared > max_magnitude)
        vector_scaled_magnitude = max_magnitude*Q_rsqrt(vector_magnitude_squared);
    else if(vector_magnitude_squared < min_magnitude) 
        vector_scaled_magnitude = min_magnitude*Q_rsqrt(vector_magnitude_squared);
    else return;

    for(T& elm: vector) elm*=vector_scaled_magnitude;
}


template<typename T>
std::vector<T> NormalizeVectorCopy(std::vector<T> vector, T magnitude)
{
    float sum, inv;

    sum = 0.f;
    for(T elm: vector)
        sum+=(elm*elm);
    
    if(sum==0) return vector;

    inv = Q_rsqrt(sum)*magnitude;
    for(T& elm: vector)
        elm*=inv;

    return vector;
}

template<typename T>
void NormalizeVectorInPlace(std::vector<T>& vector, T magnitude)
{
    float sum, inv;

    sum = 0.f;
    for(T elm: vector)
        sum+=(elm*elm);

    if(sum==0) return;

    inv = Q_rsqrt(sum)*magnitude;
    for(T& elm: vector)
        elm*=inv;
}


inline float Q_rsqrt(float number)
{
    if(number==0) throw std::runtime_error("Dividing by Zero");
	long i;
	float x2, y;
	const float threehalfs = 1.5F;

	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                       // evil floating point bit level hacking
	i  = 0x5f3759df - ( i >> 1 );               // what the fuck? 
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration

	return y;
}

#endif