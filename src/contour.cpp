#include "contour.h"

#include <algorithm>
#include <numeric>
#include <tuple>
#include <cassert>
#include <iostream>

namespace capp {

    Contour::Contour() {
    }

    Contour::Contour(size_type n) : std::vector<CVector>(n){
    }

//    Contour::Contour(const Contour& c, int startIndex, int count) : std::vector<CVector>(c) {
//        // TODO make bounding rect now?
//    }

    void Contour::equalize(unsigned int size_) {
        if (size_ > size()) {
            equalizeUp(size_);
        } else if (size_ < size()) {
            equalizeDown(size_);
        }
        // else nothing
    }

    void Contour::equalizeUp(unsigned int size_) {
        Contour storage(*this); // deep copy
        clear();
        reserve(size_);
        for (unsigned int i = 0; i < size_; i++) { // FIXME check that.
            double index = 1. * i * storage.size() / size_;
            unsigned int j = index; // integer part
            double k = index - j; // decimal part
            if (j == storage.size() - 1) {
                push_back(storage.at(j));
            } else {
                push_back(storage.at(j) * (1 - k) + storage.at(j + 1) * k);
            }
        }
    }

    void Contour::equalizeDown(unsigned int size_) { // FIXME check that.
        Contour storage(*this); // deep copy
        clear();
        reserve(size_);
        for (unsigned int i = 0; i < size_; i++) {
            push_back(CVector(0, 0));
        }
        auto storageIt = storage.begin();
        for (unsigned int i = 0; i < storage.size(); i++) {  // FIXME Could do better than that, since we're using doubles!!
            at(i * size_ / storage.size()) += *storageIt;
            storageIt++;
        }
    }

    double Contour::length() const {
        double res = 0;
        for (auto const& vect : *this) {
            res += std::norm(vect);
        }
        return res;
    }

    CVector Contour::maxNorm() const {
        CVector res;
        double max = 0;
        for (auto const& it : *this) {
            double squared_norm = ::pow(it.real(), 2) + ::pow(it.imag(), 2);
            if (squared_norm > max) {
                max = squared_norm;
                res = it;
            }
        }
        return res;
    }

    std::tuple<int, int, int, int> Contour::getBoundingRect() const {
        double minX = begin()->real();
        double maxX = 0;
        double minY = begin()->imag();
        double maxY = 0;
        for (auto const& it = begin(); it != end();) {
            minX = std::min(minX, it->real());
            maxX = std::max(maxX, it->real());
            minY = std::min(minY, it->imag());
            maxY = std::max(maxY, it->imag());
        }
        return std::make_tuple(minX, maxX, minY, maxY);
    }

    double Contour::diffR2(const Contour& c) {
#ifndef NDEBUG
        assert(size() == c.size());
#endif

        double maxNorm = 0;
        double diffSum = 0;
        double v1;
        double v2;
        for (unsigned int i = 0; i < size(); i++) {
            v1 = std::norm(at(i));
            v2 = std::norm(c.at(i));
            maxNorm = std::max(maxNorm, v1);
            maxNorm = std::max(maxNorm, v2);
            diffSum += ::pow(v1 - v2, 2);
        }
        return 1 - diffSum / size() / ::sqrt(maxNorm);
    }

    CVector Contour::dotProduct(const Contour& c, int shift) const {
#ifndef NDEBUG
        assert(size() == c.size());
#endif
        double sumA = 0;
        double sumB = 0;
        shift = shift % c.size();
        auto it2 = c.begin() + shift;
        for (auto const& it : *this) {
            sumA += it.real() * it2->real() + it.imag() * it2->imag();
//            std::cout << "it->real " << it.real() << " it2.real " << it2->real()<< " it.imag " << it.imag() << " it2.imag " << it2->imag() << std::endl;
//            std::cout << sumA << std::endl;
            sumB += it.imag() * it2->real() - it.real() * it2->imag();
            if (it2 == c.end()) {
                it2 = c.begin();
            }
        }
        return CVector(sumA, sumB);
    }

    CVector Contour::normalizedDotProduct(const Contour& c) const {
#ifndef NDEBUG
        assert(size() == c.size());
#endif
        double sumA = 0;
        double sumB = 0;
        double norm1 = 0;
        double norm2 = 0;
        auto it2 = c.begin();
        for (auto const& it : *this) {
            sumA += it.real() * it2->real() + it.imag() * it2->imag();
            sumB += it.imag() * it2->real() - it.real() * it2->imag();
            norm1 += ::pow(it.real(), 2) + ::pow(it.imag(), 2);
            norm2 += ::pow(it2->real(), 2) + ::pow(it2->imag(), 2);
            it2++;
        }
        double k = 1. / ::sqrt(norm1 * norm2);
        return CVector(sumA * k, sumB * k);
    }

    Contour Contour::correlate(const Contour& c, size_type maxShift) const {
        if (maxShift == 0) {
            maxShift = size();
        }
#ifndef NDEBUG
        assert(size() == c.size());
#endif

        Contour res;
        res.reserve(maxShift);

        for (unsigned int i = 0; i < maxShift/2; i++) {
            res.push_back(dotProduct(c, i));
        }

        int gap = size() - maxShift;
        for (unsigned int i = maxShift / 2 + gap; i < size(); i++) {
            res.push_back(dotProduct(c, i));
        }
#ifndef NDEBUG
        assert(res.size() == maxShift);
#endif
        return res;
    }

    Contour Contour::interCorrelate(const Contour& c) const {
        return correlate(c, 0);
    }

    Contour Contour::autoCorrelate() const {
        Contour res;
        double maxNorm = 0;
        res.reserve(size() / 2);

        for (unsigned int i = 0; i < size() / 2; i++) {
            auto const& dot = dotProduct(*this, i);
            res.push_back(dot);
            maxNorm = std::max(maxNorm, ::pow(dot.real(), 2) + ::pow(dot.imag(), 2));
        }
        maxNorm = ::sqrt(maxNorm);

        for (auto value : res) {
            value /= maxNorm;
       }
        return res;
    }
}



//    CVector Contour::dotProduct(Contour c, int shift) {
//        // shift
//        CVector res = CVector(0, 0);
//#ifndef NDEBUG
//         assert(size() == c.size());
//#endif
//        std::inner_product(begin(), end(), c.begin(), res);
//        return res;
//    }
