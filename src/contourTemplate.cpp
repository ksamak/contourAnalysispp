#include <iostream>
#include "contourTemplate.h"

#define PRINT_CONTOUR \
        std::cout << "template size " << _contour.size() << ": " << std::endl;\
        for (const auto& vec : _contour) {\
            std::cout << vec << " ";\
        }\
        std::cout << std::endl;\

namespace capp {

        const int ContourTemplate::defaultTemplateSize = 30;
        const std::vector<int> ContourTemplate::waveletFilters1 = { 1,  1,  1,  1};
        const std::vector<int> ContourTemplate::waveletFilters2 = {-1, -1,  1,  1};
        const std::vector<int> ContourTemplate::waveletFilters3 = {-1,  1,  1, -1};
        const std::vector<int> ContourTemplate::waveletFilters4 = {-1,  1, -1,  1};

    ContourTemplate::ContourTemplate() {
    }

    ContourTemplate::ContourTemplate(Contour c, double sourceArea, int templateSize) :
        _contour(c),
        _contourNorm(_contour.length()),
        _sourceArea(sourceArea) {

        _contour.equalize(templateSize);

        _autoCorrelated = _contour.autoCorrelate();
        calculateAutoCorrelationDescriptors();
        _contourNorm = _contour.length();
    }

    bool ContourTemplate::empty() const {
        return _contour.empty();
    }

    void ContourTemplate::clear() {
        _contour.clear();
    }

    const Contour& ContourTemplate::getContour() const {
        return _contour;
    }

    double ContourTemplate::getContourNorm() const {
        return _contourNorm;
    }

    const Contour& ContourTemplate::getAutoCorrelation() const {
        return _contour;
    }

    void ContourTemplate::calculateAutoCorrelationDescriptors() {
        double sum1 = 0;
        double sum2 = 0;
        double sum3 = 0;
        double sum4 = 0;

        for (unsigned int i = 0; i < _contour.size() / 2; i++) {
            double v = std::norm(_autoCorrelated.at(i));
            int j = i * 4. / (_contour.size() / 2.);
            sum1 += waveletFilters1.at(j) * v;
            sum2 += waveletFilters2.at(j) * v;
            sum3 += waveletFilters3.at(j) * v;
            sum4 += waveletFilters4.at(j) * v;
            //std::cout << "autocorr.at(" << i << ") = " << _autoCorrelated.at(i) << "   j " << j << "     v " << v << std::endl;
        }
        _autoCorrelationDescriptor1 = 100 * sum1 / _contour.size();
        _autoCorrelationDescriptor2 = 100 * sum2 / _contour.size();
        _autoCorrelationDescriptor3 = 100 * sum3 / _contour.size();
        _autoCorrelationDescriptor4 = 100 * sum4 / _contour.size();
    }

    std::tuple<int, int, int, int> ContourTemplate::getDescriptors() const {
        return std::make_tuple(_autoCorrelationDescriptor1, _autoCorrelationDescriptor2, _autoCorrelationDescriptor3, _autoCorrelationDescriptor4);
    }

    double ContourTemplate::getSourceArea() const {
        return _sourceArea;
    }

    void ContourTemplate::setName(const std::string& name) {
        _name = name;
    }

    const std::string& ContourTemplate::getName() const {
        return _name;
    }
}
