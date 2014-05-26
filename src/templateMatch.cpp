#include "contourTemplate.h"
#include "templateMatch.h"

namespace capp {

    TemplateMatch::TemplateMatch(double rate, ContourTemplate tem, ContourTemplate sample, double angle) :
        _rate(rate),
        _template(tem),
        _sample(sample),
        _angle(angle) {
        }

    bool TemplateMatch::empty() const {
        return _template.empty();
    }

    double TemplateMatch::getRate() const {
        return _rate;
    }

    const ContourTemplate& TemplateMatch::getTemplate() const {
        return _template;
    }

    const ContourTemplate& TemplateMatch::getSample() const {
        return _sample;
    }

    double TemplateMatch::getAngle() const {
        return _angle;
    }
    double TemplateMatch::getScale() const {
        return ::sqrt(_sample.getSourceArea() / _template.getSourceArea());
    }
};
