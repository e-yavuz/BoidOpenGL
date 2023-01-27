#include <utility>
#include <vector>
#include <iostream>
#include <set>


typedef std::vector<std::pair<size_t, size_t>* > Bucket;
struct point_bucket;
void base_split(point_bucket& base, size_t max_size, std::vector<point_bucket>& tree);

struct point_bucket
{
    point_bucket(size_t x, size_t y, size_t x_mid, size_t y_mid) : x{x}, y{y}, x_mid{x_mid}, y_mid{y_mid} {}

    point_bucket(size_t x, size_t y, size_t x_mid, size_t y_mid, std::vector<std::pair<size_t, size_t> >& bucket) : x{x}, y{y}, x_mid{x_mid}, y_mid{y_mid}
    {
        this->bucket.reserve(bucket.size());
        for(auto& elm: bucket)
            this->bucket.push_back(&elm);
    }

    size_t x;
    size_t y;
    size_t x_mid;
    size_t y_mid;
    Bucket bucket;
};

int main()
{
    std::vector<std::pair<size_t, size_t> > points;
    size_t length = 500;
    size_t width = 500;

    points.emplace_back(std::make_pair(126, 126));
    points.emplace_back(std::make_pair(126, 126));
    points.emplace_back(std::make_pair(126, 126));
    points.emplace_back(std::make_pair(126, 126));
    points.emplace_back(std::make_pair(126, 126));

    points.emplace_back(std::make_pair(0, 0));
    points.emplace_back(std::make_pair(0, 0));
    points.emplace_back(std::make_pair(0, 0));
    points.emplace_back(std::make_pair(0, 0));
    points.emplace_back(std::make_pair(0, 0));



    std::vector<point_bucket> tree;
    point_bucket base(0, 0, length/2, width/2, points);
    base_split(base, 4, tree);
}


//TODO: fix division rounding errors
void base_split(point_bucket& base, size_t max_size, std::vector<point_bucket>& tree)
{
    //Empty starting buck et
    if(base.bucket.empty()) 
        return;

    //Max resolution
    if(base.y_mid < 4 || base.x_mid < 4)
    {
        tree.push_back(base);
        return;
    }

    size_t new_x_mid = base.x_mid/2;
    size_t new_y_mid = base.y_mid/2;

    point_bucket NE(base.x + base.x_mid, base.y, new_x_mid, new_y_mid);
    point_bucket SW(base.x, base.y + base.y_mid, new_x_mid, new_y_mid);
    point_bucket SE(base.x + base.x_mid, base.y + base.y_mid, new_x_mid, new_y_mid);

    Bucket base_extra_bucket;
    if(base.bucket.size() > max_size)
    {
        for(auto iter = base.bucket.rbegin(); iter < base.bucket.rend(); iter++)
        {
            std::pair<size_t, size_t>* elm = *iter;

            if(elm->first > base.x + base.x_mid && elm->second > base.y + base.y_mid)
                SE.bucket.push_back(elm);
            else if(elm->first > base.x + base.x_mid)
                NE.bucket.push_back(elm);
            else if(elm->second > base.y + base.y_mid)
                SW.bucket.push_back(elm);
            else
                base_extra_bucket.push_back(elm);

            base.bucket.pop_back();
        }
    }

    for(auto iter = base_extra_bucket.rbegin(); iter < base_extra_bucket.rend(); iter++)
        base.bucket.push_back(*iter);

    base.x_mid = new_x_mid;
    base.y_mid =  new_y_mid;

    if(!base.bucket.empty()) 
        base.bucket.size() > max_size ? base_split(base, max_size, tree):tree.push_back(base);
    if(!NE.bucket.empty()) 
        NE.bucket.size() > max_size ? base_split(NE, max_size, tree):tree.push_back(NE);
    if(!SW.bucket.empty()) 
        SW.bucket.size() > max_size ? base_split(SW, max_size, tree):tree.push_back(SW);
    if(!SE.bucket.empty()) 
        SE.bucket.size() > max_size ? base_split(SE, max_size, tree):tree.push_back(SE);
}