#include "gui/features/gnc/trim/TrimController.hpp"

#include "messaging/SimMessageClient.hpp"

#include <algorithm>
#include <cmath>

namespace gui {
TrimController::TrimController(app::SimMessageClient &client,
    gnc::TrimRequest &request, bool &resultOpen, bool &residualOpen,
    bool &inProgress)
    : client_(client), request_(request), resultOpen_(resultOpen),
      residualOpen_(residualOpen), inProgress_(inProgress) {}

void TrimController::OnEvent(const TrimRequested &event) {
  client_.RunTrim(event.request, event.fromCurrentState);
}

void TrimController::OnEvent(const TrimRequestValueChanged &event) {
  if (!std::isfinite(event.value)) {
    return;
  }
  switch (event.field) {
  case TrimRequestField::Mode:
    request_.mode = static_cast<gnc::TrimMode>(
        std::clamp(static_cast<int>(event.value), 0, 2));
    break;
  case TrimRequestField::CalibratedAirspeedMps:
    request_.calibratedAirspeedMps = event.value;
    break;
  case TrimRequestField::AltitudeAslM:
    request_.altitudeAslM = event.value;
    break;
  case TrimRequestField::FlightPathAngleRad:
    request_.flightPathAngleRad = event.value;
    break;
  }
}

void TrimController::OnEvent(const TrimExecutionRequested &event) {
  if (inProgress_) {
    return;
  }
  inProgress_ = true;
  OnEvent(TrimRequested{request_, event.fromCurrentState});
  resultOpen_ = true;
  residualOpen_ = true;
  inProgress_ = false;
}

void TrimController::OnEvent(const TrimViewStateChanged &event) {
  resultOpen_ = event.resultOpen;
  residualOpen_ = event.residualOpen;
}
} // namespace gui
