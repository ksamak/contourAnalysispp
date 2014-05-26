#include "contourTemplate.h"
#include "templateFinder.h"

#include <iostream> // TODO remove
#include <math.h>

namespace capp {

    TemplateFinder::TemplateFinder() :
        _minACF(0.96),
        _minICF(0.85),
        _check_min_ACF(true),
        _check_min_ICF(true),
        _maxCorrelationAngle(M_PI*4), // pi(90) by default
        _maxACFDescriptorDeviation(2200) {
    }

    void TemplateFinder::setMinACF(double min) {
        _minACF = min;
    }

    double TemplateFinder::getMinACF() const {
        return _minACF;
    }

    void TemplateFinder::setMinICF(double min) {
        _minICF = min;
    }

    double TemplateFinder::getMinICF() const {
        return _minICF;
    }

    void TemplateFinder::setSearchParameters(double angle, bool checkACF, bool checkICF, int maxACFDeviation) {
        _check_min_ACF = checkACF;
        _check_min_ICF = checkICF;
        _maxCorrelationAngle = angle;
        _maxACFDescriptorDeviation = maxACFDeviation;
    }

    TemplateMatch TemplateFinder::findTemplate(std::vector<ContourTemplate> candidates, ContourTemplate target) {
        double rate = 0;
        double angle = 0;
        CVector intercorrelation(0, 0);
        ContourTemplate res;

        double descTgt1; // target descriptors
        double descTgt2;
        double descTgt3;
        double descTgt4;
        std::tie(descTgt1, descTgt2, descTgt3, descTgt4) = target.getDescriptors();

        double desc1; // candidates descriptors
        double desc2;
        double desc3;
        double desc4;

        for (auto const& candidate : candidates) {
            std::tie(desc1, desc2, desc3, desc4) = candidate.getDescriptors();
//            std::cout << "descriptors & stuff: " << std::endl;
//            std::cout << "d:" << descTgt1 << " d2:" << desc1 << " => " << ::abs(desc1 - descTgt1) << " : " << static_cast<int>(::abs(desc1 - descTgt1) > _maxACFDescriptorDeviation) << std::endl;
//            std::cout << "d:" << descTgt2 << " d2:" << desc2 << " => " << ::abs(desc2 - descTgt2) << " : " << static_cast<int>(::abs(desc2 - descTgt2) > _maxACFDescriptorDeviation) << std::endl;
//            std::cout << "d:" << descTgt3 << " d2:" << desc3 << " => " << ::abs(desc3 - descTgt3) << " : " << static_cast<int>(::abs(desc3 - descTgt3) > _maxACFDescriptorDeviation) << std::endl;
//            std::cout << "d:" << descTgt4 << " d2:" << desc4 << " => " << ::abs(desc4 - descTgt4) << " : " << static_cast<int>(::abs(desc4 - descTgt4) > _maxACFDescriptorDeviation) << std::endl;
//            if (::abs(desc1 - descTgt1) > _maxACFDescriptorDeviation || // FIXME restore
//                ::abs(desc2 - descTgt2) > _maxACFDescriptorDeviation ||
//                ::abs(desc3 - descTgt3) > _maxACFDescriptorDeviation ||
//                ::abs(desc4 - descTgt4) > _maxACFDescriptorDeviation) {
//                std::cout << "FAILED the Descriptor test" << std::endl;
//                continue;
//            }
            double r = 0;
            if (_check_min_ACF) {
                r = std::norm(candidate.getAutoCorrelation().normalizedDotProduct(target.getAutoCorrelation()));
                if (r < _minACF) {
//                    std::cout << "FAILED the MIN_ACF test" << std::endl;
                    continue;
                }
            }
            if (_check_min_ICF) {
                intercorrelation = candidate.getContour().interCorrelate(target.getContour()).maxNorm();
                r = std::norm(intercorrelation) / std::norm(candidate.getContourNorm()) * std::norm(target.getContourNorm());
                if (r < _minICF) {
//                    std::cout << "FAILED the MIN_ICF test" << std::endl;
                    continue;
                }
                if (std::abs(std::arg(intercorrelation) * 2 * M_PI) > _maxCorrelationAngle) {
//                    std::cout << "FAILED the MAX angle test(max was:" << _maxCorrelationAngle << ", found: " << std::abs(std::arg(intercorrelation) * 2 * M_PI) << ")" << std::endl;
                    continue;
                }
                std::cout << "deviation1: " << ::abs(desc1 - descTgt1) << std::endl;
                std::cout << "deviation2: " << ::abs(desc2 - descTgt2) << std::endl;
                std::cout << "deviation3: " << ::abs(desc3 - descTgt3) << std::endl;
                std::cout << "deviation4: " << ::abs(desc4 - descTgt4) << std::endl;
                std::cout << std::endl;
            }
            if (r >= rate) {
                rate = r;
                res = candidate;
                angle = std::arg(intercorrelation) * 2 * M_PI;
            }
        }
        // ignore antipattern
        if (!res.empty() and res.getName() == "antipattern") {
            res.clear();
        }
        return TemplateMatch(rate, res, target, angle);
    }
}
