#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <msgpack.hpp>

#include <cstdint>
#include <string>
#include <vector>

// Test msgpack serialization/deserialization used by AirsimRpcClient
// These are unit tests — no AirSim connection needed.

TEST_CASE("msgpack-rpc request packing") {
    msgpack::sbuffer sbuf;
    msgpack::packer<msgpack::sbuffer> pk(&sbuf);

    // Pack a request: [type=0, msgid=1, "ping", []]
    pk.pack_array(4);
    pk.pack(uint32_t{0});       // type: request
    pk.pack(uint32_t{1});       // msgid
    pk.pack(std::string("ping"));
    pk.pack_array(0);           // no params

    auto oh = msgpack::unpack(sbuf.data(), sbuf.size());
    auto& arr = oh.get().via.array;

    CHECK(arr.size == 4);
    CHECK(arr.ptr[0].as<uint32_t>() == 0);
    CHECK(arr.ptr[1].as<uint32_t>() == 1);
    CHECK(arr.ptr[2].as<std::string>() == "ping");
    CHECK(arr.ptr[3].via.array.size == 0);
}

TEST_CASE("msgpack-rpc response unpacking") {
    // Simulate a response: [type=1, msgid=1, nil, true]
    msgpack::sbuffer sbuf;
    msgpack::packer<msgpack::sbuffer> pk(&sbuf);
    pk.pack_array(4);
    pk.pack(uint32_t{1});       // type: response
    pk.pack(uint32_t{1});       // msgid
    pk.pack_nil();              // error: nil (success)
    pk.pack(true);              // result

    auto oh = msgpack::unpack(sbuf.data(), sbuf.size());
    auto& arr = oh.get().via.array;

    CHECK(arr.size == 4);
    CHECK(arr.ptr[0].as<uint32_t>() == 1);
    CHECK(arr.ptr[2].type == msgpack::type::NIL);
    CHECK(arr.ptr[3].as<bool>() == true);
}

TEST_CASE("msgpack-rpc error response") {
    msgpack::sbuffer sbuf;
    msgpack::packer<msgpack::sbuffer> pk(&sbuf);
    pk.pack_array(4);
    pk.pack(uint32_t{1});
    pk.pack(uint32_t{42});
    pk.pack(std::string("method not found"));
    pk.pack_nil();

    auto oh = msgpack::unpack(sbuf.data(), sbuf.size());
    auto& arr = oh.get().via.array;

    CHECK(arr.ptr[2].type != msgpack::type::NIL);
    CHECK(arr.ptr[2].as<std::string>() == "method not found");
    CHECK(arr.ptr[3].type == msgpack::type::NIL);
}

TEST_CASE("pack variadic params like call() template") {
    // Emulate: call("simGetImage", "front_center", 0, "", false)
    msgpack::sbuffer sbuf;
    msgpack::packer<msgpack::sbuffer> pk(&sbuf);
    pk.pack_array(4);
    pk.pack(std::string("front_center"));
    pk.pack(0);
    pk.pack(std::string(""));
    pk.pack(false);

    auto oh = msgpack::unpack(sbuf.data(), sbuf.size());
    auto& arr = oh.get().via.array;

    CHECK(arr.size == 4);
    CHECK(arr.ptr[0].as<std::string>() == "front_center");
    CHECK(arr.ptr[1].as<int>() == 0);
    CHECK(arr.ptr[2].as<std::string>().empty());
    CHECK(arr.ptr[3].as<bool>() == false);
}

TEST_CASE("pack pose map as AirSim expects") {
    msgpack::sbuffer sbuf;
    msgpack::packer<msgpack::sbuffer> pk(&sbuf);

    pk.pack_map(2);
    pk.pack("position");
    pk.pack_map(3);
    pk.pack("x_val"); pk.pack(1.0);
    pk.pack("y_val"); pk.pack(2.0);
    pk.pack("z_val"); pk.pack(-3.0);
    pk.pack("orientation");
    pk.pack_map(4);
    pk.pack("w_val"); pk.pack(1.0);
    pk.pack("x_val"); pk.pack(0.0);
    pk.pack("y_val"); pk.pack(0.0);
    pk.pack("z_val"); pk.pack(0.0);

    auto oh = msgpack::unpack(sbuf.data(), sbuf.size());
    auto& obj = oh.get();

    CHECK(obj.type == msgpack::type::MAP);
    auto& map = obj.via.map;
    CHECK(map.size == 2);

    // Verify position
    CHECK(map.ptr[0].key.as<std::string>() == "position");
    auto& pos = map.ptr[0].val.via.map;
    CHECK(pos.size == 3);

    // Find x_val in position map
    bool found_x = false;
    for (uint32_t i = 0; i < pos.size; ++i) {
        if (pos.ptr[i].key.as<std::string>() == "x_val") {
            CHECK(pos.ptr[i].val.as<double>() == doctest::Approx(1.0));
            found_x = true;
        }
    }
    CHECK(found_x);

    // Verify orientation
    CHECK(map.ptr[1].key.as<std::string>() == "orientation");
    auto& ori = map.ptr[1].val.via.map;
    CHECK(ori.size == 4);
}

TEST_CASE("binary data handling (image response)") {
    // Simulate binary image response
    std::vector<uint8_t> fake_png = {0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A};

    msgpack::sbuffer sbuf;
    msgpack::packer<msgpack::sbuffer> pk(&sbuf);
    pk.pack_bin(fake_png.size());
    pk.pack_bin_body(reinterpret_cast<const char*>(fake_png.data()), fake_png.size());

    auto oh = msgpack::unpack(sbuf.data(), sbuf.size());
    auto& obj = oh.get();

    CHECK(obj.type == msgpack::type::BIN);
    CHECK(obj.via.bin.size == fake_png.size());
    CHECK(std::memcmp(obj.via.bin.ptr, fake_png.data(), fake_png.size()) == 0);
}
