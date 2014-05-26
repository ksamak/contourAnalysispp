#ifndef CAPP_TEMPLATE_FINDER_H
#define CAPP_TEMPLATE_FINDER_H

#include "contourTemplate.h"
#include "templateMatch.h"

#include <math.h> // use #define _USE_MATH_DEFINES if compiling on old platform

namespace capp {

    class TemplateFinder {
        private:
            double _minACF; // minimum ACF intercorrelation value;
            double _minICF; // minimum ICF intercorrelation value;

            bool _check_min_ACF;
            bool _check_min_ICF;
            double _maxCorrelationAngle;
            int _maxACFDescriptorDeviation;

        public:
            TemplateFinder();
            void setMinACF(double min); // sets the detection threshold
            double getMinACF() const;
            void setMinICF(double min); // sets the detection threshold
            double getMinICF() const;
            TemplateMatch findTemplate(std::vector<ContourTemplate> candidates, ContourTemplate target);
            void setSearchParameters(double angle, bool checkACF, bool checkICF, int maxACFDeviation);
    };
}
#endif

