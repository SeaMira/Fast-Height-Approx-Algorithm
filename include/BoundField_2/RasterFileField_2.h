#include <functional>

#include <Magick++.h>

#include "BoundField_2.h"


enum RasterInterpolationType {NEAREST,BILINEAR};

// A field that encapsulates a grayscale _img stored on the specified path.
template <typename FT>
class RasterFileField_2 : public BoundField_2<FT> {
    public:
        // max_scalar is the value to which the maximum color will be mapped.
        RasterFileField_2<FT>(const std::string &path, FT max_scalar, RasterInterpolationType interp = BILINEAR) {
            _interp = interp;
            _img.read(path);
            const ssize_t cols = _img.columns();
            const ssize_t rows = _img.rows();
            _minX = (FT)0.0;
            _minY = (FT)0.0;
            _maxX = (FT)_img.columns();
            _maxY = (FT)_img.rows();
            _scale = max_scalar / MaxMap;
        };

        FT operator()(FT x, FT y) override {
            // imagemagick coordinates locate the origin at the top-left corner instead of bottom-left
            // aka flip the y axis
            y = (_maxY-y);
            switch (_interp) {
                case NEAREST: {
                    return _scale * _img.pixelColor(x, y).quantumRed();
                }
                case BILINEAR: {
                    size_t x1 = floor(x), x2 = floor(x+1), y1 = floor(y-1), y2 = floor(y); //this is most certainly wrong
                    FT q11, q12, q21, q22;
                    q11 = _scale * _img.pixelColor(x1, y1).quantumRed();
                    q12 = _scale * _img.pixelColor(x1, y2).quantumRed();
                    q21 = _scale * _img.pixelColor(x2, y1).quantumRed();
                    q22 = _scale * _img.pixelColor(x2, y2).quantumRed();
                    //super readable
                    FT z = (1.0 / ((x2 - x1) * (y2 - y1))) * (q11 * (x2 - x) * (y2 - y) + q21 * (x - x1) * (y2 - y) + q12 * (x2 - x) * (y - y1) + q22 * (x - x1) * (y - y1));
                    return z;
                }
            }
        };

        FT minX() override { return _minX; }
        FT minY() override { return _minY; }
        FT maxX() override { return _maxX; }
        FT maxY() override { return _maxY; }

    private:
        FT _minX, _minY, _maxX, _maxY;
        FT _scale;
        RasterInterpolationType _interp;
        Magick::Image _img;
        Magick::Quantum *_pixel_cache;
};