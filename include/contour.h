#ifndef CAPP_CONTOUR_H
#define CAPP_CONTOUR_H

//#include <opencv2/core/core.hpp>

#include <tuple>
#include <vector>
#include <complex>


namespace capp {

    typedef std::complex<double> CVector; // the type for complex vector

    class Contour : public std::vector<CVector> {
        private:
            // TODO boundingRect;
            void equalizeUp(unsigned int size_);
            void equalizeDown(unsigned int size_);

        public:
            Contour();
            explicit Contour(size_type n);
//            Contour(const Contour& c, int startIndex, int count);
            // explicit Contour(int capacity); # FIXME don't forget to use .allocate instead of this

            double diffR2(const Contour& c);
            double length() const;
            void normalize();
            CVector maxNorm() const;
            CVector dotProduct(const Contour& c, int shift) const;
            CVector dotProduct(const Contour& c) const {return dotProduct(c, 0);};
            CVector normalizedDotProduct(const Contour& c) const;
            void rotate(double angle);
            Contour fourrier();
            double distance(Contour c) const;
            std::tuple<int, int, int, int> getBoundingRect() const;

            void equalize(unsigned int size); // transforms the vector to fit size.
            Contour correlate(const Contour& c, size_type maxShift) const;
            Contour interCorrelate(const Contour& c) const;
            Contour autoCorrelate() const;
    };
}
#endif
