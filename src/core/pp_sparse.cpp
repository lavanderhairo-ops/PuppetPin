#include "pp_sparse.h"

#include <algorithm>
#include <cmath>

namespace pp {

bool EnvelopeCholesky::factor(const SparseSymBuilder& A)
{
    ok_ = false;
    n_ = A.size();
    const int n = n_;
    perm_.clear();
    iperm_.clear();
    first_.clear();
    rowStart_.clear();
    L_.clear();
    if (n == 0) {
        ok_ = true;
        return true;
    }

    // Merge duplicate (i, j) contributions; entries are stored with i >= j.
    std::vector<SparseSymBuilder::Entry> e = A.entries();
    std::sort(e.begin(), e.end(), [](const SparseSymBuilder::Entry& a, const SparseSymBuilder::Entry& b) {
        return a.i != b.i ? a.i < b.i : a.j < b.j;
    });
    std::vector<SparseSymBuilder::Entry> m;
    m.reserve(e.size());
    for (const auto& x : e) {
        if (x.i < 0 || x.j < 0 || x.i >= n || x.j >= n) continue;
        if (!m.empty() && m.back().i == x.i && m.back().j == x.j)
            m.back().v += x.v;
        else
            m.push_back(x);
    }

    std::vector<std::vector<int>> adj(n);
    for (const auto& x : m)
        if (x.i != x.j && x.v != 0.0) {
            adj[x.i].push_back(x.j);
            adj[x.j].push_back(x.i);
        }
    std::vector<int> deg(n);
    for (int i = 0; i < n; ++i) deg[i] = (int)adj[i].size();

    // Reverse Cuthill-McKee, starting each component from a pseudo-peripheral node.
    std::vector<int> compMark(n, -1), dist(n, -1), order;
    std::vector<char> placed(n, 0);
    order.reserve(n);
    std::vector<int> comp, queue, nbrs;
    for (int seed = 0; seed < n; ++seed) {
        if (compMark[seed] >= 0) continue;
        comp.assign(1, seed);
        compMark[seed] = seed;
        for (size_t q = 0; q < comp.size(); ++q)
            for (int nb : adj[comp[q]])
                if (compMark[nb] < 0) {
                    compMark[nb] = seed;
                    comp.push_back(nb);
                }
        int start = comp[0];
        for (int v : comp)
            if (deg[v] < deg[start]) start = v;
        int ecc = -1;
        for (int iter = 0; iter < 6; ++iter) {
            for (int v : comp) dist[v] = -1;
            queue.assign(1, start);
            dist[start] = 0;
            int maxd = 0;
            for (size_t q = 0; q < queue.size(); ++q) {
                const int v = queue[q];
                maxd = std::max(maxd, dist[v]);
                for (int nb : adj[v])
                    if (dist[nb] < 0) {
                        dist[nb] = dist[v] + 1;
                        queue.push_back(nb);
                    }
            }
            if (maxd <= ecc) break;
            ecc = maxd;
            int cand = -1;
            for (int v : comp)
                if (dist[v] == maxd && (cand < 0 || deg[v] < deg[cand])) cand = v;
            if (cand < 0 || cand == start) break;
            start = cand;
        }
        queue.assign(1, start);
        placed[start] = 1;
        for (size_t q = 0; q < queue.size(); ++q) {
            const int v = queue[q];
            order.push_back(v);
            nbrs.clear();
            for (int nb : adj[v])
                if (!placed[nb]) {
                    placed[nb] = 1;
                    nbrs.push_back(nb);
                }
            std::sort(nbrs.begin(), nbrs.end(), [&](int a, int b) { return deg[a] != deg[b] ? deg[a] < deg[b] : a < b; });
            queue.insert(queue.end(), nbrs.begin(), nbrs.end());
        }
    }
    perm_.assign(order.rbegin(), order.rend());
    iperm_.assign(n, 0);
    for (int k = 0; k < n; ++k) iperm_[perm_[k]] = k;

    // Envelope structure.
    first_.resize(n);
    for (int i = 0; i < n; ++i) first_[i] = i;
    for (const auto& x : m) {
        const int a = iperm_[x.i], b = iperm_[x.j];
        const int r = std::max(a, b), c = std::min(a, b);
        first_[r] = std::min(first_[r], c);
    }
    rowStart_.resize(n + 1);
    size_t total = 0;
    for (int i = 0; i < n; ++i) {
        rowStart_[i] = total;
        total += (size_t)(i - first_[i] + 1);
    }
    rowStart_[n] = total;
    L_.assign(total, 0.0);
    for (const auto& x : m) {
        const int a = iperm_[x.i], b = iperm_[x.j];
        const int r = std::max(a, b), c = std::min(a, b);
        L_[rowStart_[r] + (size_t)(c - first_[r])] += x.v;
    }

    // Row-oriented envelope Cholesky.
    for (int i = 0; i < n; ++i) {
        const int fi = first_[i];
        double* Li = L_.data() + rowStart_[i];  // Li[k - fi] = L(i, k)
        for (int j = fi; j < i; ++j) {
            const int fj = first_[j];
            const double* Lj = L_.data() + rowStart_[j];
            const int k0 = std::max(fi, fj);
            double s = Li[j - fi];
            const double* a = Li + (k0 - fi);
            const double* b = Lj + (k0 - fj);
            for (int k = 0, cnt = j - k0; k < cnt; ++k) s -= a[k] * b[k];
            Li[j - fi] = s / Lj[j - fj];
        }
        double s = Li[i - fi];
        for (int k = 0, cnt = i - fi; k < cnt; ++k) s -= Li[k] * Li[k];
        if (!(s > 1e-300)) return false;
        Li[i - fi] = std::sqrt(s);
    }
    ok_ = true;
    return true;
}

void EnvelopeCholesky::solve(const double* b, double* x) const
{
    const int n = n_;
    if (n == 0) return;
    std::vector<double> y(n);
    for (int k = 0; k < n; ++k) y[k] = b[perm_[k]];
    for (int i = 0; i < n; ++i) {
        const int fi = first_[i];
        const double* Li = L_.data() + rowStart_[i];
        double s = y[i];
        for (int k = fi; k < i; ++k) s -= Li[k - fi] * y[k];
        y[i] = s / Li[i - fi];
    }
    for (int i = n - 1; i >= 0; --i) {
        const int fi = first_[i];
        const double* Li = L_.data() + rowStart_[i];
        const double xi = y[i] / Li[i - fi];
        y[i] = xi;
        for (int k = fi; k < i; ++k) y[k] -= Li[k - fi] * xi;
    }
    for (int k = 0; k < n; ++k) x[perm_[k]] = y[k];
}

}  // namespace pp
