#pragma once
#include "models/mapping/room_map.hpp"

namespace vista::models {
/// Geometric intensity surfel, NOT a camera-trained/photometric 3DGS model.
struct GaussianMapPoint {
    RoomMapPoint center;
    // Symmetric world covariance: xx, xy, xz, yy, yz, zz (metres squared).
    std::array<float,6> covariance{};
    float opacity{0.85F};
};
class GaussianMapViewSource : public RoomMapViewSource {
public:
    virtual std::vector<GaussianMapPoint> select_splats(const RoomMapViewRequest&) const = 0;
    RoomMapFrame select_view(const RoomMapViewRequest& request) const final {
        RoomMapFrame frame;
        for(const auto& point:select_splats(request)) frame.points.push_back(point.center);
        return frame;
    }
};
} // namespace vista::models
