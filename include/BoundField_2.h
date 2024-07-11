#pragma once

// Abstract class describing a 2D scalar field which can be queried at any point within the bounds [(MIN_X,MIN_Y),(MAX_X,MAX_Y)].
// Such as a raster image, a interpolated point cloud, or a scalar function.
template <typename FT>
class BoundField_2 {
    public:
        // Get the scalar sample at the specified coordinates.
        // Querying outside the bounds is unspecified behavior.
        virtual FT operator()(FT x, FT y) = 0;

        virtual FT minX() = 0, minY() = 0, maxX() = 0, maxY() = 0;
};