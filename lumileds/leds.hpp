
/*
(x0, y0)--------| 
|               |
|               |
|-------------(x1, y1)
*/
class SegmentCoordinates {
    public:
        SegmentCoordinates(int x0, int y0, int x1, int y1): x0_(x0), y0_(y0), x1_(x1), y1_(y1) {}
        int x0_, y0_, x1_, y1_;
};

typedef std::pair<int, int> pair_k;