#include "app/Application.hpp"
#include "app/SimWorker.hpp"

#include "gui/GUI.hpp"
#include "messaging/SimMessageClient.hpp"
#include "sim/runtime/SimRuntime.hpp"

#include <cassert>
#include <iostream>
#include <utility>

Application::Application(std::unique_ptr<gui::GUI> gui,
    std::unique_ptr<sim::SimRuntime> simRuntime)
    : gui_(std::move(gui)) {
  if (simRuntime != nullptr) {
    simWorker_ = std::make_unique<app::SimWorker>(guiToSimQueue_,
        simToGuiQueue_,
        simToGuiTelemetryQueue_,
        std::move(simRuntime));
    simMessageClient_ = std::make_unique<app::SimMessageClient>(guiToSimQueue_,
        simToGuiQueue_,
        simToGuiTelemetryQueue_);
  }
}

Application::Application() = default;
Application::~Application() { Shutdown(); }

bool Application::Run(const volatile std::sig_atomic_t &running) {
  AssertGuiThread();
  if (!Initialize()) {
    Shutdown();
    return false;
  }

  const bool succeeded = RunMainLoop(running);
  Shutdown();
  return succeeded;
}

bool Application::RunMainLoop(const volatile std::sig_atomic_t &running) {
  AssertGuiThread();

  while (running && !gui_->ShouldClose()) {
    gui_->PollPlatformEvents();
    DrainSimEvents();
    if (simWorker_->HasFailed() || !running || gui_->ShouldClose()) {
      break;
    }
    gui_->Tick();
  }

  DrainSimEvents();
  if (simWorker_->HasFailed()) {
    std::cerr << "Simulation worker failed: " << simWorker_->GetLastError()
              << '\n';
    return false;
  }
  return true;
}

bool Application::Initialize() {
  AssertGuiThread();
  if (gui_ == nullptr || simWorker_ == nullptr
      || simMessageClient_ == nullptr) {
    std::cerr << "Application requires GUI, simulation worker, and message "
                 "client instances\n";
    return false;
  }
  gui_->SetSimMessageClient(simMessageClient_.get());
  const bool workerStarted = simWorker_->Start();
  DrainSimEvents();
  if (!workerStarted) {
    std::cerr << "Failed to initialize simulation worker: "
              << simWorker_->GetLastError() << '\n';
    return false;
  }
  if (!gui_->Initialize()) {
    std::cerr << "Failed to initialize GUI\n";
    return false;
  }
  return true;
}

void Application::DrainSimEvents() {
  AssertGuiThread();
  simToGuiQueue_.Drain();
  simToGuiTelemetryQueue_.Drain();
}

void Application::Shutdown() {
  AssertGuiThread();
  if (shutdown_) {
    return;
  }
  shutdown_ = true;
  if (simWorker_ != nullptr) {
    simWorker_->RequestStop();
    simWorker_->Join();
    simWorker_.reset();
  }
  simToGuiQueue_.Close();
  simToGuiTelemetryQueue_.Close();
  simToGuiQueue_.Drain();
  simToGuiTelemetryQueue_.Drain();
  if (gui_ != nullptr) {
    gui_->Shutdown();
  }
}

void Application::AssertGuiThread() const {
  assert(std::this_thread::get_id() == guiThreadId_
         && "Application GUI lifecycle must run on its owning thread.");
}
