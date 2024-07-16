#include <functional>

#include "BoundField_2.h"

template <typename FT>
class FunctionField_2 : public BoundField_2<FT> {
    public:
        FunctionField_2<FT>(std::function<FT(FT, FT)> scalar_func, FT minX, FT minY, FT maxX, FT maxY) {
            _func = scalar_func;
            _minX = minX;
            _minY = minY;
            _maxX = maxX;
            _maxY = maxY;
        };

        FT operator()(FT x, FT y) override {
            return _func(x, y);
        };

        FT const minX() override { return _minX; }
        FT const minY() override { return _minY; }
        FT const maxX() override { return _maxX; }
        FT const maxY() override { return _maxY; }

    private:
        std::function<FT(FT, FT)> _func;
        FT _minX, _minY, _maxX, _maxY;
};