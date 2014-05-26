#ifndef CAPP_CONTOUR_TEMPLATE_H
#define CAPP_CONTOUR_TEMPLATE_H

#include "contour.h"

#include <vector>
#include <tuple>

namespace capp {

    class ContourTemplate {
        private:
            static const int defaultTemplateSize;
            static const std::vector<int> waveletFilters1;
            static const std::vector<int> waveletFilters2;
            static const std::vector<int> waveletFilters3;
            static const std::vector<int> waveletFilters4;

            std::string _name;
            Contour _contour;
            double _contourNorm;
            Contour _autoCorrelated;
            double _sourceArea;
            int _autoCorrelationDescriptor1;
            int _autoCorrelationDescriptor2;
            int _autoCorrelationDescriptor3;
            int _autoCorrelationDescriptor4;

            void calculateAutoCorrelationDescriptors();

        public:
//            ContourTemplate(std::vector<CVector> points, double sourceArea, int templateSize);
            ContourTemplate();
            ContourTemplate(Contour c, double sourceArea, int templateSize = defaultTemplateSize);

            bool empty() const;
            void clear();
            const Contour& getContour() const;
            double getContourNorm() const;
            const Contour& getAutoCorrelation() const;
            std::tuple<int, int, int, int> getDescriptors() const;
            double getSourceArea() const;
            void setName(const std::string& name);
            const std::string& getName() const;
    };
}
#endif

