#ifndef CAPP_TEMPLATE_MATCH_H
#define CAPP_TEMPLATE_MATCH_H

#include "contourTemplate.h"

namespace capp {

    class TemplateMatch {
        private:
            double _rate;
            ContourTemplate _template;
            ContourTemplate _sample;
            double _angle;

        public:
            TemplateMatch(double rate, ContourTemplate tem, ContourTemplate sample, double angle);
            bool empty() const;
            double getRate() const;
            const ContourTemplate& getTemplate() const;
            const ContourTemplate& getSample() const;
            double getAngle() const;
            double getScale() const;
    };
}
#endif



