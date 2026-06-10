#pragma once
#include <cstdint>
#include <unordered_map>

namespace PhysicsEngine {

struct CachedLambdas {
    uint32_t id;
    float lambda_normal;
    float lambda_friction1;
    float lambda_friction2;
};

struct PairCache {
    CachedLambdas contacts[8];
    std::size_t num_contacts = 0;

    // linear scan on id, we only have 8 for now so it's quicker than hashing
    bool find(uint32_t id, CachedLambdas &out) const {
        for (std::size_t i = 0; i < num_contacts; i++) {
            if (contacts[i].id == id) {
                out = contacts[i];
                return true;
            }
        }
        return false;
    }

    void insert(const CachedLambdas &entry) {
        if (num_contacts < 8)
            contacts[num_contacts++] = entry;
    }
};

//  ordered body pair (smaller index first)
struct PairKey {
    std::size_t a, b;
    bool operator==(const PairKey &o) const { return a == o.a && b == o.b; }
};

struct PairKeyHash {
    std::size_t operator()(const PairKey &k) const {
        return (k.a + k.b) * (k.a + k.b + 1) / 2 + k.b;
    }
};

inline PairKey make_pair_key(std::size_t a, std::size_t b) {
    return (a < b) ? PairKey{a, b} : PairKey{b, a};
}

using ContactCacheMap = std::unordered_map<PairKey, PairCache, PairKeyHash>;

} // namespace PhysicsEngine
