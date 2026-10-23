// Puppet Pin - sparse symmetric positive definite solver.
//
// Envelope (skyline) Cholesky factorisation with reverse Cuthill-McKee ordering.
// Mesh Laplacian-style systems have a small envelope after RCM, so this is fast and
// completely deterministic, which keeps renders identical across machines.
#pragma once

#include <cstddef>
#include <vector>

namespace pp {

class SparseSymBuilder {
public:
    explicit SparseSymBuilder(int n = 0) : n_(n) {}
    void reset(int n) { n_ = n; entries_.clear(); }
    int size() const { return n_; }
    // Adds v to A(i,j) and, for i != j, to A(j,i).
    void add(int i, int j, double v)
    {
        if (i < j) { const int t = i; i = j; j = t; }
        entries_.push_back({i, j, v});
    }
    struct Entry { int i, j; double v; };
    const std::vector<Entry>& entries() const { return entries_; }

private:
    int n_;
    std::vector<Entry> entries_;
};

class EnvelopeCholesky {
public:
    // Factorises A. Returns false if A is not (numerically) positive definite.
    bool factor(const SparseSymBuilder& A);
    // Solves A x = b. b and x may alias. Thread-safe (const, no shared scratch).
    void solve(const double* b, double* x) const;
    int size() const { return n_; }
    bool ok() const { return ok_; }
    size_t envelopeSize() const { return L_.size(); }

private:
    int n_ = 0;
    bool ok_ = false;
    std::vector<int> perm_;   // new index -> old index
    std::vector<int> iperm_;  // old index -> new index
    std::vector<int> first_;  // first column stored in each (permuted) row
    std::vector<size_t> rowStart_;
    std::vector<double> L_;
};

}  // namespace pp
