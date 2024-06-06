#include<cmath>

class circle
{
    private:
        double radius;
        double pi = 3.1416;
    public:
        circle();
        circle(double radius);
        ~circle();

        void set(double x);

        double calculateArea();

        double calculateCircumference();

        template<typename T>
        bool pointInCircle(T x, T y){
            return std::pow(x,2) - std::pow(y,2) < std::pow(this->radius, 2);
        };
};



