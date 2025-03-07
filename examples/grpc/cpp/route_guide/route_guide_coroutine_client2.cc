/**
 * @file route_guide_coroutine_client2.cc
 * @brief Added coroutine implementation.
 *
 * The code in this file is based upon the implementation in route_guide_callback_client.cc.
 * The helper classes (Reader, Recorder; Chatter) that were orginally defined inside the functions that use them,
 * are now defined at the global level. This makes the functions shorter.
 *
 * A coroutine version has been added for ListFeatures; it is called ListFeaturesCo.
 * In contrast to the version in route_guide_coroutine_client.cc, ListFeaturesCo now contains a loop that prints
 * the received features inside this loop instead of in the callback function ReaderCo::OnReadDone.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

/*
 *
 * Copyright 2021 gRPC authors.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <chrono>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>

#include "helper.h"

#include <grpc/grpc.h>
#include <grpcpp/alarm.h>
#include <grpcpp/channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/security/credentials.h>
#ifdef BAZEL_BUILD
#include "examples/protos/route_guide.grpc.pb.h"
#else
#include "route_guide.grpc.pb.h"
#endif

#include <corolib/print.h>
#include <corolib/commservice.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using routeguide::Feature;
using routeguide::Point;
using routeguide::Rectangle;
using routeguide::RouteGuide;
using routeguide::RouteNote;
using routeguide::RouteSummary;

#include "eventqueuethr.h"

EventQueueThrFunctionVoidVoid eventQueueThr;

using namespace corolib;

const int NR_INTERACTIONS = 5;

Point MakePoint(long latitude, long longitude) {
  Point p;
  p.set_latitude(latitude);
  p.set_longitude(longitude);
  return p;
}

Feature MakeFeature(const std::string& name, long latitude, long longitude) {
  Feature f;
  f.set_name(name);
  f.mutable_location()->CopyFrom(MakePoint(latitude, longitude));
  return f;
}

RouteNote MakeRouteNote(const std::string& message, long latitude,
                        long longitude) {
  RouteNote n;
  n.set_message(message);
  n.mutable_location()->CopyFrom(MakePoint(latitude, longitude));
  return n;
}

class Reader : public grpc::ClientReadReactor<Feature> {
public:
    Reader(RouteGuide::Stub* stub, float coord_factor,
        const routeguide::Rectangle& rect)
        : coord_factor_(coord_factor) {
        stub->async()->ListFeatures(&context_, &rect, this);
        StartRead(&feature_);
        StartCall();
    }
    void OnReadDone(bool ok) override {
        if (ok) {
            std::cout << "Found feature called " << feature_.name() << " at "
                << feature_.location().latitude() / coord_factor_ << ", "
                << feature_.location().longitude() / coord_factor_
                << std::endl;
            StartRead(&feature_);
        }
    }
    void OnDone(const Status& s) override {
        std::unique_lock<std::mutex> l(mu_);
        status_ = s;
        done_ = true;
        cv_.notify_one();
    }
    Status Await() {
        std::unique_lock<std::mutex> l(mu_);
        cv_.wait(l, [this] { return done_; });
        return std::move(status_);
    }

private:
    ClientContext context_;
    float coord_factor_;
    Feature feature_;
    std::mutex mu_;
    std::condition_variable cv_;
    Status status_;
    bool done_ = false;
};

enum class ReaderState
{
    Value,
    Done
};

struct ReaderResult
{
    ReaderState state;
    Feature feature;
    Status status;
};

class ReaderCo : public grpc::ClientReadReactor<Feature> {
public:
    ReaderCo(RouteGuide::Stub* stub, float coord_factor,
        const routeguide::Rectangle& rect)
        : coord_factor_(coord_factor) {
        stub->async()->ListFeatures(&context_, &rect, this);
        StartRead(&feature_);
        StartCall();
    }
    void OnReadDone(bool ok) override {
        if (ok) {
            print(PRI5, "OnReadDone\n");
            ReaderResult result;
            result.state = ReaderState::Value;
            result.feature = feature_;
            eventQueueThr.push(
                [this, result]() {
                    this->completionHandler_(result);
                });
            StartRead(&feature_);
        }
    }
    void OnDone(const Status& s) override {
        print(PRI1, "OnDone\n");
        status_ = s;
        ReaderResult result;
        result.state = ReaderState::Done;
        result.status = status_;
        eventQueueThr.pushFinal(
            [this, result]() {
                this->completionHandler_(result);
            });
    }

    void setCompletionHandler(std::function<void(ReaderResult)>&& completionHandler) {
        completionHandler_ = std::move(completionHandler);
    }

private:
    ClientContext context_;
    float coord_factor_;
    Feature feature_;
    Status status_;
    std::function<void(ReaderResult)> completionHandler_;
};

class Recorder : public grpc::ClientWriteReactor<Point> {
public:
    Recorder(RouteGuide::Stub* stub, float coord_factor,
        const std::vector<Feature>* feature_list)
        : coord_factor_(coord_factor),
        feature_list_(feature_list),
        generator_(
            std::chrono::system_clock::now().time_since_epoch().count()),
        feature_distribution_(0, feature_list->size() - 1),
        delay_distribution_(500, 1500) {
        stub->async()->RecordRoute(&context_, &stats_, this);
        // Use a hold since some StartWrites are invoked indirectly from a
        // delayed lambda in OnWriteDone rather than directly from the reaction
        // itself
        AddHold();
        NextWrite();
        StartCall();
    }
    void OnWriteDone(bool ok) override {
        // Delay and then do the next write or WritesDone
        alarm_.Set(
            std::chrono::system_clock::now() +
            std::chrono::milliseconds(delay_distribution_(generator_)),
            [this](bool /*ok*/) { NextWrite(); });
    }
    void OnDone(const Status& s) override {
        std::unique_lock<std::mutex> l(mu_);
        status_ = s;
        done_ = true;
        cv_.notify_one();
    }
    Status Await(RouteSummary* stats) {
        std::unique_lock<std::mutex> l(mu_);
        cv_.wait(l, [this] { return done_; });
        *stats = stats_;
        return std::move(status_);
    }

private:
    void NextWrite() {
        if (points_remaining_ != 0) {
            const Feature& f =
                (*feature_list_)[feature_distribution_(generator_)];
            std::cout << "Visiting point "
                << f.location().latitude() / coord_factor_ << ", "
                << f.location().longitude() / coord_factor_ << std::endl;
            StartWrite(&f.location());
            points_remaining_--;
        }
        else {
            StartWritesDone();
            RemoveHold();
        }
    }
    ClientContext context_;
    float coord_factor_;
    int points_remaining_ = 10;
    Point point_;
    RouteSummary stats_;
    const std::vector<Feature>* feature_list_;
    std::default_random_engine generator_;
    std::uniform_int_distribution<int> feature_distribution_;
    std::uniform_int_distribution<int> delay_distribution_;
    grpc::Alarm alarm_;
    std::mutex mu_;
    std::condition_variable cv_;
    Status status_;
    bool done_ = false;
};

class Chatter : public grpc::ClientBidiReactor<RouteNote, RouteNote> {
public:
    explicit Chatter(RouteGuide::Stub* stub)
        : notes_{ MakeRouteNote("First message", 0, 0),
                 MakeRouteNote("Second message", 0, 1),
                 MakeRouteNote("Third message", 1, 0),
                 MakeRouteNote("Fourth message", 0, 0) },
        notes_iterator_(notes_.begin()) {
        stub->async()->RouteChat(&context_, this);
        NextWrite();
        StartRead(&server_note_);
        StartCall();
    }
    void OnWriteDone(bool /*ok*/) override { NextWrite(); }
    void OnReadDone(bool ok) override {
        if (ok) {
            std::cout << "Got message " << server_note_.message() << " at "
                << server_note_.location().latitude() << ", "
                << server_note_.location().longitude() << std::endl;
            StartRead(&server_note_);
        }
    }
    void OnDone(const Status& s) override {
        std::unique_lock<std::mutex> l(mu_);
        status_ = s;
        done_ = true;
        cv_.notify_one();
    }
    Status Await() {
        std::unique_lock<std::mutex> l(mu_);
        cv_.wait(l, [this] { return done_; });
        return std::move(status_);
    }

private:
    void NextWrite() {
        if (notes_iterator_ != notes_.end()) {
            const auto& note = *notes_iterator_;
            std::cout << "Sending message " << note.message() << " at "
                << note.location().latitude() << ", "
                << note.location().longitude() << std::endl;
            StartWrite(&note);
            notes_iterator_++;
        }
        else {
            StartWritesDone();
        }
    }
    ClientContext context_;
    const std::vector<RouteNote> notes_;
    std::vector<RouteNote>::const_iterator notes_iterator_;
    RouteNote server_note_;
    std::mutex mu_;
    std::condition_variable cv_;
    Status status_;
    bool done_ = false;
};


class RouteGuideClient : public CommService {
 public:
  RouteGuideClient(std::shared_ptr<Channel> channel, const std::string& db)
      : stub_(RouteGuide::NewStub(channel)) {
    routeguide::ParseDb(db, &feature_list_);
  }

  // GetFeature
  // ----------

  void GetFeature() {
    Point point;
    Feature feature;
    point = MakePoint(409146138, -746188906);
    GetOneFeature(point, &feature);
    point = MakePoint(0, 0);
    GetOneFeature(point, &feature);
  }

  // ListFeatures
  // ------------
  // original version

  void ListFeatures() {
    routeguide::Rectangle rect;
    Feature feature;

    rect.mutable_lo()->set_latitude(400000000);
    rect.mutable_lo()->set_longitude(-750000000);
    rect.mutable_hi()->set_latitude(420000000);
    rect.mutable_hi()->set_longitude(-730000000);
    std::cout << "Looking for features between 40, -75 and 42, -73"
              << std::endl;

    Reader reader(stub_.get(), kCoordFactor_, rect);
    Status status = reader.Await();
    if (status.ok()) {
      std::cout << "ListFeatures rpc succeeded." << std::endl;
    } else {
      std::cout << "ListFeatures rpc failed." << std::endl;
    }
  }

  // coroutine version

  async_task<void> ListFeaturesCo() {
      print(PRI1, "ListFeaturesCo - begin\n");
      routeguide::Rectangle rect;
      Feature feature;

      rect.mutable_lo()->set_latitude(400000000);
      rect.mutable_lo()->set_longitude(-750000000);
      rect.mutable_hi()->set_latitude(420000000);
      rect.mutable_hi()->set_longitude(-730000000);
      std::cout << "Looking for features between 40, -75 and 42, -73"
          << std::endl;

      ReaderCo reader(stub_.get(), kCoordFactor_, rect);
      async_operation<ReaderResult> op = start_ListFeatures(&reader);
      op.auto_reset(true);
      bool done = false;
      do {
          print(PRI5, "ListFeaturesCo: before co_await\n");
          ReaderResult result = co_await op;
          print(PRI5, "ListFeaturesCo: after co_await\n");
          if (result.state == ReaderState::Value) {
              std::cout << "Found feature called " << result.feature.name() << " at "
                  << result.feature.location().latitude() / kCoordFactor_ << ", "
                  << result.feature.location().longitude() / kCoordFactor_
                  << std::endl;
          }
          if (result.state == ReaderState::Done) {
              done = true;
              if (result.status.ok()) {
                  std::cout << "ListFeatures rpc succeeded." << std::endl;
              }
              else {
                  std::cout << "ListFeatures rpc failed." << std::endl;
              }
          }
      } while (!done);

      print(PRI1, "ListFeaturesCo - end\n");
      co_return;
  }

  int count = 0;

  async_operation<ReaderResult> start_ListFeatures(ReaderCo* pReaderCo) {
      int index = get_free_index();
      async_operation<ReaderResult> ret{ this, index };
      pReaderCo->setCompletionHandler(
          [this, index](ReaderResult result) {
                print(PRI5, "completionHandler called\n");
                this->completionHandler<ReaderResult>(index, result);
          });
      return ret;
  }

  // RecordRoute
  // -----------

  void RecordRoute() {
    Recorder recorder(stub_.get(), kCoordFactor_, &feature_list_);
    RouteSummary stats;
    Status status = recorder.Await(&stats);
    if (status.ok()) {
      std::cout << "Finished trip with " << stats.point_count() << " points\n"
                << "Passed " << stats.feature_count() << " features\n"
                << "Travelled " << stats.distance() << " meters\n"
                << "It took " << stats.elapsed_time() << " seconds"
                << std::endl;
    } else {
      std::cout << "RecordRoute rpc failed." << std::endl;
    }
  }

  // RouteChat
  // ---------

  void RouteChat() {
    Chatter chatter(stub_.get());
    Status status = chatter.Await();
    if (!status.ok()) {
      std::cout << "RouteChat rpc failed." << std::endl;
    }
  }

 private:
  bool GetOneFeature(const Point& point, Feature* feature) {
    ClientContext context;
    bool result;
    std::mutex mu;
    std::condition_variable cv;
    bool done = false;
    stub_->async()->GetFeature(
        &context, &point, feature,
        [&result, &mu, &cv, &done, feature, this](Status status) {
          bool ret;
          if (!status.ok()) {
            std::cout << "GetFeature rpc failed." << std::endl;
            ret = false;
          } else if (!feature->has_location()) {
            std::cout << "Server returns incomplete feature." << std::endl;
            ret = false;
          } else if (feature->name().empty()) {
            std::cout << "Found no feature at "
                      << feature->location().latitude() / kCoordFactor_ << ", "
                      << feature->location().longitude() / kCoordFactor_
                      << std::endl;
            ret = true;
          } else {
            std::cout << "Found feature called " << feature->name() << " at "
                      << feature->location().latitude() / kCoordFactor_ << ", "
                      << feature->location().longitude() / kCoordFactor_
                      << std::endl;
            ret = true;
          }
          std::lock_guard<std::mutex> lock(mu);
          result = ret;
          done = true;
          cv.notify_one();
        });
    std::unique_lock<std::mutex> lock(mu);
    cv.wait(lock, [&done] { return done; });
    return result;
  }

  const float kCoordFactor_ = 10000000.0;
  std::unique_ptr<RouteGuide::Stub> stub_;
  std::vector<Feature> feature_list_;
};

async_task<void> runListFeaturesCo(RouteGuideClient& guide)
{
    print(PRI1, "runListFeaturesCo\n");
    for (int i = 0; i < NR_INTERACTIONS; ++i) {
        std::cout << "-------------- ListFeatures using coroutines (1) (" << i << ") --------------" << std::endl;
        async_task<void> tlf = guide.ListFeaturesCo();
        co_await tlf;
    }
    co_return;
}

async_task<void> runListFeaturesCo2(RouteGuideClient& guide)
{
    print(PRI1, "runListFeaturesCo2\n");
    for (int i = 0; i < NR_INTERACTIONS; ++i) {
        std::cout << "-------------- ListFeatures using coroutines (2) (" << i << ") --------------" << std::endl;
        async_task<void> tlf = guide.ListFeaturesCo();
        eventQueueThr.reset();
        runEventQueue(eventQueueThr);
        //tlf.wait();
    }
    co_return;
}

int main(int argc, char** argv) {
  set_print_level(0x01);

  print(PRI1, "Entered main\n");

  // Expect only arg: --db_path=path/to/route_guide_db.json.
  std::string db = routeguide::GetDbFileContent(argc, argv);
  RouteGuideClient guide(
      grpc::CreateChannel("localhost:50051",
                          grpc::InsecureChannelCredentials()),
      db);
  
  std::cout << "-------------- GetFeature --------------" << std::endl;
  guide.GetFeature();

  for (int i = 0; i < NR_INTERACTIONS; ++i) {
      std::cout << "-------------- ListFeatures (" << i << ") --------------" << std::endl;
      guide.ListFeatures();
  }
  std::cout << "-------------- RecordRoute --------------" << std::endl;
  guide.RecordRoute();
  std::cout << "-------------- RouteChat --------------" << std::endl;
  guide.RouteChat();

  print(PRI1, "main: async_task<void> tlf1 = runListFeaturesCo(guide);\n");
  async_task<void> tlf1 = runListFeaturesCo(guide);
  for (int i = 0; i < NR_INTERACTIONS; ++i) {
      eventQueueThr.reset();
      runEventQueue(eventQueueThr);
  }
  print(PRI1, "main: tlf1.wait();\n");
  //tlf1.wait();

  print(PRI1, "main: async_task<void> tlf2 = runListFeaturesCo(guide);\n");
  async_task<void> tlf2 = runListFeaturesCo2(guide);
  print(PRI1, "main: tlf2.wait();\n");
  tlf2.wait();
#
  print(PRI1, "Leaving main\n");
  return 0;
}
