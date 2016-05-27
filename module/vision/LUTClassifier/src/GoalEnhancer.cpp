/*
 * This file is part of the Autocalibration Codebase.
 *
 * The Autocalibration Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The Autocalibration Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Autocalibration Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "LUTClassifier.h"
#include "QuexClassifier.h"
#include "utility/math/ransac/NPartiteRansac.h"
#include "utility/math/geometry/Line.h"
#include "utility/nubugger/NUhelpers.h"

namespace module {
    namespace vision {

        using message::input::Image;
        using message::vision::LookUpTable;
        using message::vision::ObjectClass;
        using message::vision::ClassifiedImage;
        using utility::math::ransac::NPartiteRansac;
        using utility::math::geometry::Line;

        using utility::nubugger::drawVisionLines;

        struct GoalPOI {
            arma::vec2 midpoint;
            uint length;
        };

        struct GoalPOIModel {

            using DataPoint = GoalPOI;
            static constexpr size_t REQUIRED_POINTS = 2;

            Line line;
            std::array<uint, 2> lengths;

            GoalPOIModel() {}

            bool regenerate(const std::array<DataPoint, REQUIRED_POINTS>& pts, const arma::vec2& horizonTangent, const double& maxAngle) {
                line.setFromPoints(pts[0].midpoint, pts[1].midpoint);
                lengths[0] = pts[0].length;
                lengths[1] = pts[1].length;

                return arma::dot(line.normal, horizonTangent) > maxAngle;
            };

            double calculateError(const DataPoint& p) const {
                double dist = line.distanceToPoint(p.midpoint);
                uint d1 = std::abs(lengths[0] - p.length);
                uint d2 = std::abs(lengths[1] - p.length);

                return dist * dist + d1 * d1 + d2 * d2;
            };

            template <typename Iterator>
            void refineModel(Iterator& /*begin*/, Iterator& /*end*/, const double& /*threshold*/) {
            }
        };

        void LUTClassifier::enhanceGoals(const Image& image, const LookUpTable& lut, ClassifiedImage<ObjectClass>& classifiedImage) {

            /*
                Here we improve the classification of goals.
                We do this by taking our course classification of the whole image
                and generating new segments where yellow was detected.
                We first generate segments above and below that are 2x the width of the segment
             */

            // Get our goal segments
            std::vector<GoalPOI> points;
            auto hSegments = classifiedImage.horizontalSegments.equal_range(ObjectClass::GOAL);
            for(auto it = hSegments.first; it != hSegments.second; ++it) {

                // Insert all our points
                if(it->second.length > GOAL_MINIMUM_RANSAC_SEGMENT_SIZE) {
                    points.push_back({{double(it->second.midpoint[0]), double(it->second.midpoint[1])}, it->second.length});
                }
            }

            // Partition our segments so that they are split between above and below the horizon
            auto split = std::partition(std::begin(points), std::end(points), [&] (const GoalPOI& point) {
                // Is the midpoint above or below the horizon?
                return classifiedImage.horizon.distanceToPoint(point.midpoint) > 0;
            });

            // Make an array of our partitions
            std::array<std::vector<GoalPOI>::iterator, GoalPOIModel::REQUIRED_POINTS + 1> iterators = {
                points.begin(),
                split,
                points.end()
            };

            // Ransac for goals
            auto models = NPartiteRansac<GoalPOIModel>::fitModels(iterators
                                                                , GOAL_RANSAC_MINIMUM_POINTS_FOR_CONSENSUS
                                                                , GOAL_RANSAC_MAXIMUM_ITERATIONS_PER_FITTING
                                                                , GOAL_RANSAC_MAXIMUM_FITTED_MODELS
                                                                , GOAL_RANSAC_CONSENSUS_ERROR_THRESHOLD
                                                                , classifiedImage.horizon.tangent()
                                                                , GOAL_MAX_HORIZON_ANGLE);


            std::vector<ClassifiedImage<ObjectClass>::Segment> newSegments;

            std::vector<std::pair<arma::ivec2, arma::ivec2>> goalLines;
            for(auto& model : models) {

                double lineMid = 0;
                double lineWidth = 0;

                // Go through our points to find our extents
                for(auto& m : model) {
                    lineMid += model.model.line.tangentialDistanceToPoint(m.midpoint);
                    lineWidth += m.length;
                }

                // Normalise our line length
                lineWidth /= std::distance(model.begin(), model.end());
                double lineHalfWidth = (lineWidth / 2) * GOAL_HORIZONTAL_EXTENSION_SCALE;
                lineMid /= std::distance(model.begin(), model.end());

                // Work out how long our line should be
                // Get our actual line width
                double lineLength = ((lineWidth * model.model.line.normal[0]) / GOAL_WIDTH_HEIGHT_RATIO) * GOAL_VERTICAL_EXTENSION_SCALE;

                double minTangent = lineMid - lineLength / 2;
                double maxTangent = lineMid + lineLength / 2;

                // TODO clamp minTangent and maxTangent to the edges of the screen
                // Get the min and max for intersecting with top and bottom of the screen

                double jump = (maxTangent - minTangent) / double(GOAL_LINE_INTERSECTIONS);
                for(auto d = minTangent; d <= maxTangent; d += jump) {

                    // Get our centre point
                    arma::vec2 p = model.model.line.pointFromTangentialDistance(d);

                    if((p[1] > int(image.height) - 1) || (p[1] < 0)) {
                        continue;
                    }

                    // Start and end
                    arma::ivec2 s({ std::max(0, int(std::round(p[0] - lineHalfWidth))), int(std::round(p[1]))});
                    arma::ivec2 e({ std::min(int(image.width) - 1, int(std::round(p[0] + lineHalfWidth))), int(std::round(p[1]))});

                    if(e[0] > 0) {
                        auto segments = quex->classify(image, lut, s, e);
                        newSegments.insert(newSegments.begin(), segments.begin(), segments.end());
                    }
                }

                insertSegments(classifiedImage, newSegments, false);
            }
        }

    }  // vision
}  // modules
