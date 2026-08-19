# C++ — the study file for section 8 of [emescent.md](emescent.md)

Written 2026-08-19. One section per checklist item: the concept at "can hold a technical
conversation" depth, the sentence an interviewer is fishing for, and an honest **`piros2` line**.
The syllabus marks this section **THE FIRST-PRIORITY GAP** — "four processes in one week turned
on it (Emesent, Arista, Anduril, the defence lane). The standing claim is **embedded firmware
C/C++, not large-scale application or STL C++**, and that wording does not change until
something is built." Per the honest-claim rule: reading this file does not move anything into
skills.md. `piros2` is **all Python** — six `ament_python` packages, zero `.cpp` files
(checked with `find` on 2026-08-19); its one real C++ contact is that `ament_cmake` is the
C++ package convention and colcon builds both kinds side by side. Every `piros2` line below
says so and points at what the rclcpp equivalent would be. Emesent's ad asks for C++14/17+ for
performance-critical, real-time robotics components on Linux, in a ROS-based architecture, on
compute-constrained hardware — every section is framed for that.

## Mental model to carry through the whole file

| Standard | What it added (the part an engineer names in the room) |
| --- | --- |
| **C++11** | the "new language": `auto`, lambdas, rvalue references + move semantics, `unique_ptr`/`shared_ptr`/`weak_ptr`, `std::thread`/`mutex`/`atomic`/`<chrono>`, `constexpr`, range-`for`, variadic templates, `nullptr`, `enum class`, `override`/`final`, `= default`/`= delete`, `static_assert`, `noexcept`, `std::array`, `unordered_map`, `std::function` |
| **C++14** | polish: generic lambdas (`auto` params), lambda init-capture (`[p = std::move(p)]`), `std::make_unique`, return-type deduction, relaxed `constexpr`, variable templates, `shared_timed_mutex`, digit separators `1'000` |
| **C++17** | vocabulary types: `std::optional`, `variant`, `any`, `string_view`; structured bindings, `if constexpr`, `if (init; cond)`, fold expressions, CTAD, guaranteed copy elision, inline variables, `std::filesystem`, parallel algorithms, `std::pmr`, `scoped_lock`/`shared_mutex`, `[[nodiscard]]`, `hardware_destructive_interference_size`, `std::byte` |
| **C++20** | the big four — concepts, ranges, coroutines, modules — plus `<=>`, designated initialisers, `std::span`, `std::format`, `jthread`/`stop_token`, `atomic::wait/notify`, `latch`/`barrier`/`semaphore`, `consteval`/`constinit`, `std::bit_cast`, `[[likely]]`, `source_location`, `constexpr` `vector`/`string` |
| **C++23** | `std::expected`, `std::print`, deducing `this`, `std::generator`, `mdspan`, `flat_map`, `stacktrace`, monadic `optional`, `ranges::to`, `import std;` |

Where it lands on this repo's toolchain: the dev box has **g++ 13.3.0 and CMake 3.28.3**
(checked 2026-08-19; `cppcheck`, `gdb`, `perf` installed, no `clang-tidy` or `valgrind`).
GCC 13 is essentially C++20-complete in language and library and carries the C++23 pieces that
matter here — `std::expected` since libstdc++ 12 — while `std::print` waits for GCC 14. rclcpp
itself is compiled as C++17 in Jazzy; user packages set `target_compile_features(... cxx_std_17)`
and may go higher.

## 1. C++14/17/20 feature sets and what each added

- The table above is the answer; the interviewer wants to hear the *shape*, not a list. C++11
  made the language: value semantics with move, RAII with smart pointers, closures, a memory
  model and threads in the standard. **C++14** is C++11 finished — the missing
  `make_unique`, `auto` in lambda parameters, init-capture (which is what makes move-only
  captures possible), relaxed `constexpr` (loops allowed). **C++17** is the *vocabulary*
  release — the types two libraries can agree on at an interface (`optional`, `variant`,
  `string_view`), plus compile-time branching (`if constexpr`) that retires most SFINAE, and
  guaranteed elision so `T make()` never copies. **C++20** is the *big-language* release:
  concepts (constrain templates in words the compiler can print), ranges (composable lazy
  pipelines), coroutines (language-level only — no `std::generator` until 23), modules
  (still uneven in build tools in 2026), plus `std::span` (the non-owning view of contiguous
  memory a sensor driver hands out), designated initialisers (`Point{.x = 1, .y = 2}`),
  `std::format`, `jthread`. **C++23** rounds it off with `std::expected` for error handling
  without exceptions, `std::print`, and `mdspan` for tensors.
- The job ad's "C++14/17+" means: they build with `-std=c++17` in production and expect you
  to reach for `optional`/`variant`/structured bindings by reflex, know C++20 exists, and not
  write C++98 with `new`/`delete`.
- The classic slip: attributing features to the wrong standard — `make_unique` is 14 not 11;
  `optional`/`variant`/`string_view`/structured bindings/`if constexpr` are 17;
  concepts/ranges/coroutines/modules/`span`/designated initialisers/`<=>` are 20;
  `expected` is 23.
- **Interviewer's target sentence:** "I write C++17 by default — `optional`, `variant`,
  `string_view`, structured bindings, `if constexpr` — and reach into C++20 for `span`,
  concepts and designated initialisers when the toolchain allows; C++11 gave us move
  semantics and smart pointers, C++14 finished them off."
- **`piros2` line:** not touched — every node is `ament_python`. The nearest equivalent:
  its subscriptions would be `rclcpp::Node::create_subscription<sensor_msgs::msg::CompressedImage>`
  with a lambda callback and a `std::shared_ptr<const Msg>` argument, compiled under Jazzy's
  C++17 baseline.

## 2. RAII

- *Resource Acquisition Is Initialisation*: a resource (memory, file, lock, socket, camera
  handle, GPU buffer) is acquired in a constructor and released in the destructor, so its
  lifetime *is* the owning object's scope. Destructors run in reverse declaration order on
  every exit path — normal return, `break`, exception unwinding — so cleanup is deterministic
  and cannot be forgotten. This is the single idea the rest of modern C++ hangs on:
  `unique_ptr`, `lock_guard`, `fstream`, `std::jthread` (joins in its destructor), and
  every well-designed handle type.
- Rules that follow: never write a raw `new` without a smart pointer wrapper; never write
  `mutex.lock()`/`unlock()` by hand; wrap every C API handle (`FILE*`, `int fd`, a V4L2
  device, a CUDA stream) in a struct with a destructor or a `unique_ptr` with a custom deleter.
  Destructors must not throw (they are implicitly `noexcept`; a throw during unwinding calls
  `std::terminate`).
- In rclcpp the pattern is everywhere and is a classic trap: a subscription, timer or
  publisher *is* its `shared_ptr` — let it fall out of scope (assign the result of
  `create_subscription` to a local instead of a member) and it silently unsubscribes and the
  callback never fires. `rclcpp::init`/`shutdown` bracket the process; a `Node` owns its
  entities.

```cpp
{
    std::lock_guard<std::mutex> lk(state_mutex_);   // acquire
    latest_ = std::move(msg);
}                                                     // release, on every path
```

- **Interviewer's target sentence:** "Ownership lives in constructors and destructors; if I
  see a bare `new`, `lock()`, `fopen` or `cudaMalloc` without a scope-bound owner, that's a
  leak waiting for the first exception."
- **`piros2` line:** not C++ — but the same discipline is the repo's shell-level rule:
  every session recipe's `trap … EXIT` `pkill -f`s the nodes it started ("sessions tear
  themselves down"), and Python's `with`/`try…finally` play the destructor. The rclcpp
  version of `keypoint_detector`'s subscriptions would be `rclcpp::Subscription<…>::SharedPtr`
  members held for the node's whole life.

## 3. Smart pointers: unique_ptr, shared_ptr, weak_ptr

- **`std::unique_ptr<T>`** — sole ownership, move-only, zero overhead (same size as a raw
  pointer with the default deleter; a stateful deleter grows it). Transfer with `std::move`;
  `release()` gives the raw pointer back and forgets it; `get()` borrows. Custom deleters
  wrap C handles: `std::unique_ptr<FILE, decltype(&fclose)> f(fopen(...), &fclose)`. Prefer
  `std::make_unique<T>(args)` (C++14) — exception-safe, no naked `new`.
- **`std::shared_ptr<T>`** — shared ownership through a heap *control block* holding a strong
  and a weak count, incremented/decremented **atomically** (a contended cache line — a cost
  in hot loops), two pointers wide. `std::make_shared` allocates object and block together
  (one allocation, better locality — but the memory lives until the last `weak_ptr` dies).
  Thread-safe control block, *not* a thread-safe pointee. `enable_shared_from_this` lets an
  object hand out `shared_ptr`s to itself — `rclcpp::Node` inherits it, and calling
  `shared_from_this()` in a constructor throws `bad_weak_ptr` (the reason many rclcpp
  patterns defer setup to an `init()` after construction).
- **`std::weak_ptr<T>`** — a non-owning observer of a `shared_ptr`; `.lock()` returns a
  `shared_ptr` or empty. Breaks reference cycles (parent ↔ child, node ↔ callback that
  captures the node) and expresses "I'll use it if it's still alive".
- Rule of thumb: `unique_ptr` by default; `shared_ptr` only when ownership is genuinely
  shared (rclcpp messages between intra-process subscribers, node handles); raw pointers and
  references for non-owning access in a call; `weak_ptr` for back-references and caches.

```cpp
auto frame = std::make_unique<Frame>(w, h);
queue.push(std::move(frame));       // ownership moves; `frame` is now nullptr
```

- **Interviewer's target sentence:** "`unique_ptr` is free and says 'mine'; `shared_ptr` costs
  an atomic refcount and says 'ours', so I use it only where lifetime is genuinely shared —
  which in rclcpp is the message and the node; `weak_ptr` breaks the cycles."
- **`piros2` line:** not touched. Python's refcounting is `shared_ptr` everywhere by
  default, which is why the repo never had to think about it. In rclcpp the messages
  `keypoint_detector` receives would arrive as `sensor_msgs::msg::CompressedImage::ConstSharedPtr`,
  and the transport rework's `camera_relay` fan-out would be a `unique_ptr` message
  republished intra-process without a copy.

## 4. Move semantics, rvalue references, perfect forwarding

- **Value categories:** lvalues have identity (a named variable); rvalues (prvalues,
  xvalues) are temporaries or things marked as expiring. `T&&` binds to rvalues; `std::move(x)`
  is *only a cast* to `T&&` — no move happens until a move constructor or assignment is
  chosen. A move constructor *steals* the source's resources (pointer copy + null out the
  source) instead of deep-copying; the moved-from object must be left valid-but-unspecified
  (destructible, assignable).
- **Why it matters for robotics:** a 2.7 MB image or a 45k-point cloud passed by value costs
  a memcpy; moved, it costs three pointer copies. `std::vector`, `std::string`, `unique_ptr`
  and every well-behaved type move for free. Return by value — RVO/NRVO elides the copy, and
  C++17 *guarantees* elision for prvalues, so `return std::move(local);` is a pessimisation
  (compilers warn `-Wpessimizing-move`).
- **`noexcept` on move constructors matters:** `std::vector` reallocation uses
  `std::move_if_noexcept` and falls back to *copying* if the move might throw — a silent
  10× slowdown from a missing `noexcept`.
- **Perfect forwarding:** in a deduced context `template<class T> void f(T&& x)` `T&&` is a
  *forwarding reference* (binds to anything, reference-collapsing rules); `std::forward<T>(x)`
  passes it on preserving lvalue/rvalue-ness. This is how `make_unique`, `emplace_back` and
  `std::thread`'s constructor pass arguments through without extra copies.

```cpp
template <class T, class... Args>
std::unique_ptr<T> make(Args&&... args) {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
```

- **Interviewer's target sentence:** "`std::move` is a cast; the move happens in the
  callee's constructor. I mark moves `noexcept` so vectors actually move on growth, and I
  don't `return std::move(...)` because elision already beats it."
- **`piros2` line:** not touched. The nearest thinking is numpy's copy-vs-view discipline:
  `cloud_projector` builds a structured array whose `tobytes()` *is* the `PointCloud2`
  payload — one copy at the wire, none before it. In rclcpp that would be
  `publisher->publish(std::move(cloud_msg))` on a `unique_ptr` message.

## 5. Rule of 0/3/5

- **Rule of three (C++98):** if a class needs a user-defined destructor, copy constructor
  *or* copy assignment, it almost certainly needs all three — because it owns a resource the
  compiler's memberwise copy would double-free. **Rule of five (C++11):** add the move
  constructor and move assignment, or the class silently copies where it could move.
  **Rule of zero:** the goal — own resources only through members that already follow the
  rules (`unique_ptr`, `vector`, `string`) and declare *none* of the five; the compiler
  generates correct ones.
- The subtle rule: declaring a destructor (even `= default`) *suppresses* the implicit move
  operations, and makes the implicit copies deprecated. So a class with a virtual
  `~Base() = default;` should spell out `Base(const Base&) = default;` etc. if it is meant to
  be movable. `= delete` on copies gives a move-only type (`unique_ptr` is the model).

```cpp
class Buffer {                       // rule of five, hand-rolled (teaching only —
    float* data_; size_t n_;         //  in real code, hold a std::vector: rule of zero)
public:
    explicit Buffer(size_t n) : data_(new float[n]), n_(n) {}
    ~Buffer() { delete[] data_; }
    Buffer(const Buffer& o) : data_(new float[o.n_]), n_(o.n_) { std::copy_n(o.data_, n_, data_); }
    Buffer& operator=(Buffer o) noexcept { swap(*this, o); return *this; }   // copy-and-swap
    Buffer(Buffer&& o) noexcept : data_(o.data_), n_(o.n_) { o.data_ = nullptr; o.n_ = 0; }
    friend void swap(Buffer& a, Buffer& b) noexcept { std::swap(a.data_, b.data_); std::swap(a.n_, b.n_); }
};
```

- **Interviewer's target sentence:** "Rule of zero first — if I'm writing a destructor, I'm
  writing five special members and I ask why a `unique_ptr` member wouldn't do; the
  by-value `operator=` with swap gives strong exception safety and covers both copy and move."
- **`piros2` line:** not touched. Python's `@dataclass` (used in `pose_graph.py`) is the
  rule of zero in spirit — the compiler-generated members are the right ones because the
  fields own themselves.

## 6. Const correctness

- `const T*` (pointer to const) vs `T* const` (const pointer): read declarations right-to-left.
  A `const` member function promises not to modify observable state and is the only kind
  callable on a `const` object or through a `const&`; `mutable` marks members that are
  logically-not-state (a cache, a mutex — you must lock in a `const` getter). Pass
  non-trivial arguments as `const T&`, return by value, and make locals `const` by default
  so the reader knows what varies.
- The standard library's thread-safety contract is stated in `const`: distinct threads may
  call `const` members on the same object concurrently; any non-`const` call needs external
  synchronisation. So "const-correct" is also "documents what is safe to read from another
  thread".
- `constexpr` (compile-time evaluable, C++11; loops allowed since 14; `std::vector`/`string`
  usable in constant expressions since 20), `consteval` (must run at compile time, C++20),
  `constinit` (static initialised at compile time — kills the static-initialisation-order
  fiasco, C++20). `const_cast` is a smell that means an API is wrong.
- **Interviewer's target sentence:** "Const is documentation the compiler checks — const
  member functions say what a reader may call from another thread, `const&` parameters say
  I won't copy or mutate, and `mutable` is reserved for caches and mutexes."
- **`piros2` line:** not touched — Python has no `const`. The repo's analogue is the
  "pure function" convention its docstrings insist on (`se3.py`, `mesh_fill.py`,
  `dashboard.rates`, `finish_mesh` — "Pure"), which is what made them unit-testable without a
  running node.

## 7. Templates, specialisation, SFINAE, concepts

- **Templates** are compile-time code generation: a function or class parameterised on types
  (and non-type values). Instantiation happens per distinct argument set — flexible and
  fast (everything is inlinable), at the cost of compile time, binary size and error
  messages. **Full specialisation** replaces the template for one argument set;
  **partial specialisation** (class templates only) for a family (`template<class T> struct
  Traits<T*>`). Two-phase lookup: `typename`/`template` disambiguators for dependent names.
- **SFINAE** — *substitution failure is not an error*: if substituting deduced types into a
  template's signature fails, that overload is silently discarded instead of erroring. Used
  via `std::enable_if_t<cond>`, `std::void_t`, and expression SFINAE in `decltype` to select
  overloads by property ("has `.size()`"). Powerful, unreadable, and the errors are famous.
  **C++17 `if constexpr`** removes most in-body uses (discard a branch at compile time).
- **C++20 concepts** replace the rest: named boolean predicates over types, checked at the
  call site with a readable diagnostic ("constraints not satisfied: T does not satisfy
  Number"). Standard ones: `std::integral`, `std::floating_point`, `std::invocable`,
  `std::ranges::range`. Overloading on concepts picks the most-constrained candidate.

```cpp
template <class T>
concept PointLike = requires(T p) {
    { p.x } -> std::convertible_to<float>;
    { p.y } -> std::convertible_to<float>;
    { p.z } -> std::convertible_to<float>;
};
template <PointLike P> float norm(const P& p) { return std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z); }
```

- Robotics idioms built on templates: Eigen (`Matrix<Scalar, Rows, Cols>` — expression
  templates), PCL (`PointCloud<PointT>`), rclcpp (`create_subscription<MsgT>`,
  `create_publisher<MsgT>`), `message_filters::Synchronizer<Policy>`. CRTP for static
  polymorphism where a virtual call in the hot loop is unwelcome.
- **Interviewer's target sentence:** "Templates are how the ROS and Eigen APIs stay
  zero-cost; I constrain them with concepts if I have C++20 and `if constexpr` if I have 17,
  and I only reach for `enable_if` when I'm stuck on 14."
- **`piros2` line:** not touched — Python duck-types. The message-generic idea shows up as
  the `POINT_DTYPE` structured array in `cloud_projector` (any field layout that matches the
  `PointField` list is a valid cloud); the rclcpp version is
  `sensor_msgs::PointCloud2Iterator<float>` or PCL's `PointCloud<PointXYZRGB>` template.

## 8. STL containers and their complexity guarantees

| Container | Backing | Index | Insert/erase | Notes |
| --- | --- | --- | --- | --- |
| `vector` | contiguous array | O(1) | end: amortised O(1); middle: O(n) | default choice; `reserve()`; reallocation invalidates everything; `vector<bool>` is a bit-packed trap |
| `array<T,N>` | fixed, on the stack | O(1) | — | no allocation — the real-time favourite |
| `deque` | blocks of arrays | O(1) | both ends O(1) | push_front/back keep *references* valid, iterators not |
| `list` / `forward_list` | linked nodes | — | O(1) with an iterator | one allocation per node, cache-hostile; almost never the right answer |
| `map` / `set` | red-black tree | O(log n) | O(log n) | ordered, iterators stable except the erased one |
| `unordered_map` / `unordered_set` | hash buckets | avg O(1), worst O(n) | avg O(1) | rehash invalidates iterators, not references; node-based, so a cache miss per lookup |
| `priority_queue` | heap on a vector | top O(1) | push/pop O(log n) | — |
| `string` | contiguous + SSO | O(1) | O(n) | small-string optimisation: 15 chars in libstdc++, 22 in libc++, before the heap |

- The rule that beats the table: for n up to a few thousand, a sorted `vector` with binary
  search beats `map`, and a linear scan of a `vector` beats `unordered_map` — cache lines
  win over big-O. Node-based containers pay a heap allocation and a pointer chase per
  element; contiguous ones prefetch. Reserve up front; never allocate in the loop.
- Robotics-specific: `std::array` and pre-reserved `vector`s in the control path;
  `boost::circular_buffer` or a hand-rolled ring for sensor windows; `absl::flat_hash_map` /
  `robin_hood` / `ankerl::unordered_dense` when a hash map is hot (open addressing, one cache
  line); C++23 `flat_map`.
- **Interviewer's target sentence:** "`vector` unless proven otherwise — contiguous memory
  and amortised O(1) growth beat the asymptotics of node containers at robotics sizes; I
  `reserve`, I know what invalidates iterators, and I keep node-based maps out of the hot loop."
- **`piros2` line:** not touched. `keypoint_detector`'s 10-frame descriptor window is a
  `collections.deque(maxlen=match_window)` and the dashboard's arrival timestamps are
  `deque`s — in C++ those are `boost::circular_buffer` or a fixed `std::array` ring, and the
  ORB descriptor matrix is an `Eigen`/`cv::Mat` slab, not a container of points.

## 9. STL algorithms and iterators

- Iterators generalise pointers over half-open ranges `[begin, end)`; categories — input,
  output, forward, bidirectional, random-access, and (C++20) contiguous — decide which
  algorithms are legal and how fast (`std::sort` needs random access; `std::lower_bound` is
  O(log n) only on random access). Algorithms live in `<algorithm>`/`<numeric>`: `sort`
  (introsort, O(n log n), not stable), `stable_sort`, `partial_sort`, `nth_element` (O(n)
  average — the median-of-a-cloud tool), `find_if`, `lower_bound`, `accumulate`/`reduce`,
  `transform`, `partition`, `unique`, `rotate`, `minmax_element`, `all_of`/`any_of`, `iota`,
  `inner_product`/`transform_reduce`, `sample`, `clamp`. The erase-remove idiom
  (`v.erase(std::remove_if(...), v.end())`) became `std::erase_if(v, pred)` in C++20.
- **C++17 execution policies** (`std::execution::par`, `par_unseq`) parallelise
  `sort`/`transform`/`reduce` (libstdc++ needs TBB). **C++20 ranges**:
  `std::ranges::sort(v)`, projections (`ranges::sort(pts, {}, &Point::z)`), lazy views
  (`v | views::filter(pred) | views::transform(f) | views::take(10)`) — no temporaries,
  composable, and the code reads like the intent. C++23 `ranges::to<std::vector>()`
  materialises.
- Why an interviewer asks: hand-written loops hide intent and bugs (`<` vs `<=`); algorithms
  name what happens, are tested, and vectorise. Knowing that `nth_element` is the
  robust-statistics tool and that `lower_bound` on a sorted timestamp vector is how a TF
  buffer or a bag index finds a stamp is the "has used it" signal.
- **Interviewer's target sentence:** "No raw loops where an algorithm names the operation —
  `nth_element` for a median, `lower_bound` on sorted stamps, `transform_reduce` for a dot
  product — and ranges when I have C++20 so the pipeline reads as a pipeline."
- **`piros2` line:** not touched — the repo's "algorithms" are numpy: `cloud_projector` is
  "vectorised over the whole image; no loops", the depth aligner uses a rolling median, and
  `pose_graph.py` is dense `np.linalg.solve`. The C++ shape is Eigen for the linear algebra
  and `<algorithm>` for the bookkeeping.

## 10. Lambdas and std::function

- A **lambda** is an anonymous closure type generated by the compiler: captures become
  members, the body becomes `operator()`. Capture by value `[=]`/`[x]`, by reference
  `[&]`/`[&x]`, `[this]`, and (C++14) *init-capture* `[p = std::move(ptr)]` — the only way
  to move a `unique_ptr` into a closure. `mutable` lets the body modify by-value captures;
  C++14 generic lambdas take `auto` parameters; C++17 `[*this]` copies the object and
  lambdas are implicitly `constexpr`-capable; C++20 template lambdas `[]<class T>(T x)`.
  A capture-less lambda converts to a plain function pointer (C callbacks). Calling a lambda
  through its own type is inlinable and free.
- **`std::function<R(Args...)>`** is a *type-erased* callable holder: it can store any
  callable with that signature (lambda, function pointer, functor, `std::bind` result) at
  the cost of an indirect call and, for captures larger than its small buffer (~16 bytes in
  libstdc++), a heap allocation. It requires the callable to be copyable — a lambda that
  captured a `unique_ptr` won't fit until C++23's `std::move_only_function`. Use it at
  boundaries (storing callbacks); use templates/`auto` for hot paths.
- rclcpp callbacks are stored type-erased (`AnySubscriptionCallback` holds a
  `std::function`-like variant of the accepted signatures); the idiomatic modern form is a
  lambda `[this](sensor_msgs::msg::Image::ConstSharedPtr msg) { on_frame(msg); }` rather
  than `std::bind(&Node::on_frame, this, std::placeholders::_1)`.

```cpp
auto job = [frame = std::move(frame), this]() mutable { integrate(std::move(frame)); };
worker_.post(std::move(job));      // move-only closure; needs a move-aware queue
```

- **Interviewer's target sentence:** "Lambdas are zero-cost closures — I capture by move
  with init-capture when the closure owns something; `std::function` is type erasure with a
  possible allocation, so it's fine as a stored callback and wrong in a per-point loop."
- **`piros2` line:** `dashboard.py` subscribes with `lambda msg: self.on_feed('camera', msg)`
  — a Python closure doing exactly the rclcpp lambda's job (bind a feed name to a shared
  handler). No `std::function` cost model applies in Python.

## 11. std::optional, variant, string_view, structured bindings

- **`std::optional<T>`** (C++17): value-or-nothing without a heap allocation or a sentinel;
  `if (auto p = find_pose(t))`, `*p`, `p->x`, `.value()` (throws `bad_optional_access`),
  `.value_or(default)`; C++23 monadic `and_then`/`transform`/`or_else`. The right return type
  for "may not have one" — a lookup miss, a lost track — replacing out-params and magic
  values.
- **`std::variant<A, B, C>`** (C++17): a type-safe tagged union — exactly one alternative
  alive, `std::visit` with an overloaded-lambda set dispatches at compile time,
  `std::get_if<A>(&v)` peeks, `.index()`. Sum types for messages, states, "one of these
  sensor packets"; `std::monostate` for empty. Never `valueless_by_exception` in practice.
- **`std::string_view`** (C++17): non-owning `{pointer, length}` over characters, pass by
  value, no null terminator (don't hand `.data()` to a C API), O(1) `substr`. **The classic
  bug:** binding a view to a temporary — `std::string_view sv = name + "_frame";` dangles at
  the semicolon. Same family: `std::span<T>` (C++20) for contiguous non-owning ranges — the
  right parameter type for "a slice of the point buffer".
- **Structured bindings** (C++17): `auto [it, inserted] = map.emplace(k, v);`,
  `for (auto& [key, val] : map)`, `auto [R, t] = kabsch(a, b);` — works for arrays, tuple-like
  types and plain aggregates. Combined with `if (auto it = m.find(k); it != m.end())`
  (init-statement) it removes a whole class of scope-leaked temporaries.
- **Interviewer's target sentence:** "`optional` for maybe-a-value, `variant` for
  one-of-these, `string_view`/`span` for borrowed slices with the dangling rule in mind, and
  structured bindings so a returned pair reads as two named things — that's the C++17
  vocabulary I write in."
- **`piros2` line:** not touched. Python `None` returns and tuple unpacking play both roles
  — `keypoint_detector` returns "could not estimate" as a flag and the tests unpack
  `(R, ok)`-style tuples; `tsdf_mesher.poll_refresh` treats a `None` poll result as "not
  ready yet", which in C++ is `std::optional<MeshResult>` from a non-blocking `try_pop`.

## 12. Exceptions and error-handling strategy, expected

- **How C++ exceptions cost:** the Itanium ABI's "zero-cost" model means the non-throwing
  path pays nothing (tables, not checks), and the throwing path pays a lot and unpredictably
  — allocation of the exception object, unwinder table lookup, `dl_iterate_phdr` under a
  global lock in older glibc — microseconds to far more, unbounded. That is why exceptions are
  *not real-time safe* and why control loops and firmware compile `-fno-exceptions`. Rules:
  throw by value, catch by `const&`; destructors don't throw; `noexcept` on moves and
  anything the standard library calls; the three guarantees — nothrow, strong (all or
  nothing, e.g. copy-and-swap), basic (invariants hold, state unspecified).
- **Alternatives:** error codes (`std::error_code`/`std::errc` — cheap, ignorable, the
  `[[nodiscard]]` attribute helps), `std::optional` when the only error is "none",
  `absl::StatusOr<T>`, and **`std::expected<T, E>` (C++23)** — a value *or* an error, checked
  by the caller, no unwinding, monadic chaining (`and_then`, `transform`, `or_else`),
  `std::unexpected(err)` to construct the error side; `tl::expected` is the pre-23 drop-in.
  Strategy that most robotics codebases converge on: exceptions for programmer errors and
  construction/config failure (an invalid parameter *should* abort startup), `expected` /
  status for expected runtime failures (no transform yet, no frame, sensor timeout), and
  never across a real-time boundary or a C ABI.

```cpp
std::expected<Pose, LookupError> lookup(const Frame& f, Stamp t);
if (auto p = lookup(f, t)) integrate(*p); else RCLCPP_WARN(log, "%s", p.error().what());
```

- rclcpp throws on misuse (`rclcpp::exceptions::ParameterNotDeclaredException`,
  `InvalidParametersException`) and tf2 throws for lookup failures
  (`tf2::LookupException`, `ExtrapolationException`, `ConnectivityException` — all
  `tf2::TransformException`), so a `try { buffer.lookupTransform(...) } catch (const
  tf2::TransformException& ex)` around every lookup is the canonical rclcpp code you'd
  write, and `canTransform` first is how you avoid paying the throw in a loop.
- **Interviewer's target sentence:** "Exceptions are free until thrown and then unbounded,
  so I keep them for startup and programmer errors and out of the control path; runtime
  failures return `expected` or `optional`, and I `catch (const tf2::TransformException&)`
  because tf2 gives me no choice."
- **`piros2` line:** in Python, exceptions *are* the strategy — `cloud_projector` and
  `tsdf_mesher` wrap `lookup_transform` in `except TransformException`, the mesher catches
  `ImportError` (open3d lazy import), `RuntimeError` from the worker and — telling —
  `rclpy._rclpy_pybind11.RCLError` at publish-during-shutdown. No `expected` anywhere; the
  C++ port would return `std::expected` from `poll()`.

## 13. Threading: std::thread, mutexes, condition variables, atomics

- **`std::thread`** (C++11) runs a callable; you *must* `join()` or `detach()` before it is
  destroyed or `std::terminate` fires — **`std::jthread`** (C++20) joins in its destructor
  and carries a `std::stop_token` for cooperative cancellation. `std::async`/`future`/
  `promise` for one-shot results. `thread_local` for per-thread state.
- **Mutexes:** `std::mutex`, `recursive_mutex` (a smell), `timed_mutex`, `shared_mutex`
  (C++17; readers-writer — `shared_lock` for readers, `unique_lock` for the writer),
  `shared_timed_mutex` (C++14). Lock through RAII: `lock_guard` (simple), `unique_lock`
  (movable, needed by condition variables), `scoped_lock` (C++17, locks several without
  deadlock). Keep critical sections tiny; never hold a lock across a callback or I/O; lock
  order discipline for more than one.
- **`std::condition_variable`:** wait for a predicate under a `unique_lock` —
  `cv.wait(lk, [&]{ return !queue.empty() || stop; })` — always with the predicate, because
  of *spurious wakeups* and lost notifications; `notify_one` after releasing the lock is
  slightly cheaper. C++20 alternatives: `std::latch`, `std::barrier`,
  `std::counting_semaphore`, `atomic::wait/notify`.
- **`std::atomic<T>`:** lock-free (check `is_lock_free()`; 64-bit ints and pointers are on
  x86-64/ARM64, 16-byte structs usually not) load/store/RMW (`fetch_add`,
  `compare_exchange_weak/strong`) with a memory-order argument (§14). A `bool`/counter/
  sequence number shared between threads is an atomic; a struct is a mutex.
- **In rclcpp** the threading model is the executor's: `rclcpp::spin(node)` is a
  `SingleThreadedExecutor` (all callbacks serialised, no locks needed but one slow callback
  starves the rest); `MultiThreadedExecutor` runs callbacks concurrently, and **callback
  groups** — `MutuallyExclusive` (default: never two callbacks of the group at once) vs
  `Reentrant` — decide which may overlap; Jazzy adds an experimental `EventsExecutor`.
  Anything shared between groups needs a mutex or an atomic. Long work (meshing,
  optimisation) belongs on its own `std::thread`/`jthread` fed by a queue so the executor
  keeps servicing subscriptions.
- **Interviewer's target sentence:** "Executor thread for callbacks, worker threads for
  anything that takes longer than a frame period, a queue between them, `unique_lock` +
  condition variable with a predicate, atomics for flags and counters, and I know which
  callback group each subscription is in before I share state."
- **`piros2` line:** the repo learned this the hard way in Python: `mesh_worker.py` records
  that decimation-plus-completion (12–21 s) run inline starved TSDF integration, run on a
  *thread* still did — Open3D holds the GIL — so `MeshFinisher` spawns a **separate process**
  (spawn context, arrays pickled through a pipe, non-blocking `poll()`, `os.nice(10)`). In
  C++ that is exactly one `std::jthread` and an SPSC queue — no GIL, no pickling; the lesson
  transfers, the mechanism is the point of the C++ port.

## 14. Memory model, data races, false sharing

- **The C++11 memory model** defines what a multi-threaded program means: *sequenced-before*
  within a thread, *synchronizes-with* between threads (an acquire load that reads the value
  of a release store, a mutex unlock/lock, thread creation/join), composing into
  *happens-before*. Two conflicting accesses (at least one write, same location) not
  ordered by happens-before are a **data race**, and a data race is **undefined behaviour**
  — not "one of the two values" but "the compiler may assume it never happens".
  `volatile` is not a synchronisation tool; it is for memory-mapped I/O.
- **Memory orders** on atomics: `seq_cst` (default; a single global order — safest, dearest
  on ARM), `acquire`/`release` (the pair a message-passing pattern needs: writer publishes
  data then `store(release)`, reader `load(acquire)` then reads data — everything before the
  release is visible after the acquire), `acq_rel` for RMW, `relaxed` (atomicity only — a
  statistics counter). Fences: `std::atomic_thread_fence`.
- **Hardware matters:** x86 is TSO (stores are the only reordering, so racy code often
  "works"); **ARMv8 is weakly ordered** — a race that never shows on the dev box shows on the
  Pi, a Jetson, or the ARM SoC in a field payload. That is a robotics-specific reason to run
  TSan and to be explicit about ordering.
- **False sharing:** two variables written by two threads sitting in the same 64-byte cache
  line ping-pong the line between cores under MESI coherence — no data race, correct
  results, 10–100× slower. Fix with `alignas(std::hardware_destructive_interference_size)`
  (C++17, 64 on these targets) or padding — the SPSC ring's head and tail (§15) are the
  textbook case. **True sharing** (both threads use the same data) is what mutex-protected
  hot state costs; the fix there is to share less.
- **Interviewer's target sentence:** "A data race is UB, not a stale read; I use
  release/acquire pairs for message passing, relaxed for counters, `seq_cst` when I can't
  prove otherwise, and I pad the producer's and consumer's indices onto separate cache lines
  because false sharing is invisible in a code review and obvious in `perf`."
- **`piros2` line:** not touched — the GIL serialises Python, and the one concurrency
  boundary in the repo (mesh worker) crosses a process pipe with copies, so no shared memory
  exists to race on. The transferable fact is the target: the Pi 5's Cortex-A76 is the
  weakly-ordered ARM where an rclcpp port's races would surface first.

## 15. Lock-free structures, ring buffers, SPSC queues

- **Lock-free** = some thread makes progress in bounded steps regardless of others being
  suspended (no lock to be preempted while holding); **wait-free** = every thread does.
  Built from atomics and CAS loops; the classic pitfalls are the ABA problem (a CAS sees the
  same value after a remove-and-reinsert — solved with tagged pointers/generation counters
  or hazard pointers), memory reclamation, and the fact that "lock-free" is not "faster" —
  it is *latency-bounded*, which is what a real-time producer needs.
- **The SPSC ring buffer** is the one lock-free structure every real-time engineer should
  be able to write: one producer, one consumer, a fixed power-of-two array, `head` written
  only by the consumer, `tail` only by the producer, each read by the other with acquire and
  written with release; empty when `head == tail`, full when `(tail + 1) & mask == head`
  (one slot sacrificed). No CAS at all — single-writer per index is what makes it trivially
  correct. Bounded, allocation-free after construction, drop-or-overwrite policy on full.
  Multi-producer/consumer variants (MPMC — Vyukov's bounded queue, `moodycamel::ConcurrentQueue`,
  `boost::lockfree::queue`) need CAS and are far subtler; SPSC libraries:
  `boost::lockfree::spsc_queue`, `folly::ProducerConsumerQueue`, `moodycamel::ReaderWriterQueue`.

```cpp
template <class T, size_t N>            // N power of two; T trivially copyable/movable
class Spsc {
    static_assert((N & (N - 1)) == 0);
    std::array<T, N> buf_;
    alignas(64) std::atomic<size_t> head_{0};   // consumer writes
    alignas(64) std::atomic<size_t> tail_{0};   // producer writes
public:
    bool push(T v) {
        auto t = tail_.load(std::memory_order_relaxed);
        if (((t + 1) & (N - 1)) == head_.load(std::memory_order_acquire)) return false; // full
        buf_[t] = std::move(v);
        tail_.store((t + 1) & (N - 1), std::memory_order_release);
        return true;
    }
    bool pop(T& out) {
        auto h = head_.load(std::memory_order_relaxed);
        if (h == tail_.load(std::memory_order_acquire)) return false;             // empty
        out = std::move(buf_[h]);
        head_.store((h + 1) & (N - 1), std::memory_order_release);
        return true;
    }
};
```

- Where it lives in a robot: driver ISR/thread → processing thread (IMU samples at 200–1000
  Hz), sensor thread → logger, control loop → telemetry — every "hard-timed producer, soft
  consumer" edge. It is also what DDS `KEEP_LAST` history depth *is* conceptually.
- **Interviewer's target sentence:** "For one producer and one consumer I don't need CAS —
  a power-of-two ring with an acquire/release pair on each index, head and tail on separate
  cache lines, bounded and allocation-free; anything multi-producer I take from a library
  and read the ABA section first."
- **`piros2` line:** not touched at the C++ level. The concept sits in the repo as
  `deque(maxlen=…)` windows, the DDS `KEEP_LAST` depth-1 subscriptions ("latest-wins", the
  dashboard's deliberate design), and the mesh worker's `multiprocessing` queue — all
  library-provided, none lock-free. The rclcpp port of `camera_relay` → detector/mesher
  hand-off is where the ring above would go.

## 16. Allocation: heap vs stack, custom allocators, memory pools

- **Stack:** bump-pointer, freed at scope exit, hot in cache, bounded by thread stack size
  (8 MB main, often 1–2 MB for spawned threads — a 1280×720 float image does *not* go on
  it). **Heap:** `malloc`/`new` — general-purpose, takes locks in the allocator, may fault
  in pages, fragments, has *unbounded worst-case latency*; the reason it is banned in
  real-time paths. Real-time recipe: allocate everything at init, `mlockall(MCL_CURRENT |
  MCL_FUTURE)` so pages never swap, pre-fault the stack, and run the loop with zero
  `new`/`malloc` (verify with a counting allocator, `ltrace`, or ASan's `malloc` hooks).
- **Custom allocators:** every STL container takes an `Allocator` template parameter;
  **C++17 `std::pmr`** (polymorphic memory resources) makes it runtime — `pmr::vector<T>`
  over a `monotonic_buffer_resource` (bump, free-all-at-once: perfect per-frame arenas) or
  `unsynchronized_pool_resource` (fixed-size pools). **Memory pools / free lists** give O(1)
  bounded alloc/free for fixed-size objects (messages, nodes); TLSF (two-level segregated
  fit) is the classic bounded-time general allocator and is what ROS 2's `realtime_support`
  ships (`tlsf_cpp`) with an allocator template you can hand to rclcpp publishers/
  subscriptions/executors. Placement `new` constructs into memory you already own.
- **The other cost of `new`:** heap objects scatter — data-oriented design (struct of
  arrays, contiguous buffers) is as much about allocation strategy as about cache lines.
  `std::vector::reserve`, `std::string` SSO, `std::array`, and fixed-capacity types
  (`etl::vector`, `boost::static_vector`, `absl::InlinedVector`) are the daily tools.
- **Interviewer's target sentence:** "Allocation is fine at startup and forbidden in the
  loop: pre-size, `mlockall`, and if a container must grow at runtime give it a pool or a
  `pmr` arena — I check with a counting allocator, not by reading the code."
- **`piros2` line:** not touched — CPython allocates per object and numpy per array;
  `cloud_projector` builds `np.empty(n, dtype=POINT_DTYPE)` per cloud and `VoxelMap` is
  array-backed. In rclcpp the point buffer would be one `reserve`d `std::vector<Point>`
  reused per frame, or a `pmr` arena reset per callback.

## 17. Undefined behaviour

- **UB** is the contract's escape hatch: the standard imposes no requirements, so the
  compiler may assume it never happens and optimise on that assumption. That is why UB is
  *not* "a crash" — it is a program that passes tests at `-O0` and misbehaves at `-O2`, on
  another compiler, or on ARM. The everyday sources: signed integer overflow (so
  `if (i + 1 < i)` is deleted), out-of-bounds indexing, use-after-free and dangling
  references (returning a reference to a local; `string_view` of a temporary),
  dereferencing null, uninitialised reads, **data races**, shifting by ≥ the width or a
  negative amount, strict-aliasing violations (`*reinterpret_cast<float*>(&u32)` — use
  `memcpy` or C++20 `std::bit_cast`), misaligned access (fine on x86, a fault on some
  ARM/DSP), modifying a `const` object, ODR violations across TUs, unsequenced modifications
  (`i = i++`), calling a pure virtual from a constructor, `std::vector` iterator use after
  reallocation, `std::sort` with a comparator that isn't a strict weak ordering.
- Distinguish: **implementation-defined** (documented choice — `sizeof(int)`, right shift of
  negative until C++20 fixed it as arithmetic), **unspecified** (one of a set — evaluation
  order of function arguments), **UB** (anything). Robotics relevance: packed sensor
  structs, byte-swapping, float bit-punching (RViz's packed `rgb` float!) and hand-rolled
  serialisation are exactly where UB hides.
- Defence: `-Wall -Wextra -Wpedantic -Wshadow -Wconversion`, UBSan and ASan in CI (§20),
  `-fwrapv` if you *need* wrapping, `std::bit_cast`/`memcpy` for type-punning, `.at()` in
  debug builds, `gsl::span`/`std::span` instead of pointer+length, and the mindset that a
  test passing does not prove absence of UB.
- **Interviewer's target sentence:** "UB isn't a crash, it's a licence for the optimiser —
  signed overflow, aliasing and data races are the ones that pass every test and fail in
  the field; I punch floats with `bit_cast`, index with `span`, and run UBSan in CI."
- **`piros2` line:** not touched — Python raises where C++ would silently continue. The
  one place the repo does C-style bit-punching is `cloud_projector`'s packed
  `0x00RRGGBB`-into-a-`float32` `rgb` field, done via numpy dtype views (safe in Python); the
  C++ version is `std::bit_cast<float>(packed)`, and a `reinterpret_cast` there is the
  strict-aliasing UB an interviewer might ask you to spot.

## 18. Build systems: CMake, targets, linking

- **Modern CMake is target-based:** `add_library`/`add_executable` create targets;
  `target_link_libraries(tgt PUBLIC dep)` propagates *usage requirements* (include dirs,
  defines, flags, other libs) — `PRIVATE` for implementation-only, `PUBLIC` for what appears
  in your headers, `INTERFACE` for header-only. `target_include_directories`,
  `target_compile_features(tgt PUBLIC cxx_std_17)`, `target_compile_options`;
  `find_package(Eigen3 REQUIRED)` imports targets like `Eigen3::Eigen`; generator
  expressions `$<BUILD_INTERFACE:…>`/`$<INSTALL_INTERFACE:…>`. Never `include_directories()`
  or `link_libraries()` globally; never hard-code `-I` paths. Build types: `Debug`,
  `Release` (-O3 -DNDEBUG), `RelWithDebInfo` (-O2 -g — what you ship and profile),
  `MinSizeRel`. `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON` for clang-tidy/clangd. Ninja over Make.
  `FetchContent` for source deps; `install()` + export sets for consumers.
- **Linking:** static `.a` (copied in, no runtime dep, order matters on the link line —
  dependents before dependencies) vs shared `.so` (loaded by `ld.so` at start via
  `DT_NEEDED`, found through `RPATH`/`RUNPATH`, `LD_LIBRARY_PATH`, `ldconfig` cache;
  `ldd`/`readelf -d`/`nm -D` are the tools). Symbol visibility (`-fvisibility=hidden` +
  export macros — rclcpp's `RCLCPP_PUBLIC`), the ODR across shared objects, `--as-needed`,
  `dlopen` for plugins (`pluginlib`, image_transport plugins, RMW implementations — the
  `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` switch is a `dlopen` at runtime).
- **ROS 2 wraps it:** an `ament_cmake` package's `CMakeLists.txt` calls `find_package(ament_cmake
  REQUIRED)`, `find_package(rclcpp REQUIRED)`, builds targets, links them
  (Jazzy tutorials use `ament_target_dependencies(node rclcpp sensor_msgs)`; the plain
  modern-CMake `target_link_libraries(node PUBLIC rclcpp::rclcpp)` form also works and is
  what later distros prefer), `install(TARGETS … DESTINATION lib/${PROJECT_NAME})`, and ends
  with `ament_package()`; `rclcpp_components_register_node` makes a composable node.
  **colcon** drives the whole workspace: it topologically sorts packages by `package.xml`,
  builds `ament_cmake` packages with CMake and `ament_python` ones with `setup.py`, and
  `--symlink-install` links Python/launch/config sources so edits need no rebuild;
  `--cmake-args -DCMAKE_BUILD_TYPE=Release` and `--packages-select` are the daily flags.

```cmake
cmake_minimum_required(VERSION 3.16)
project(piros2_edges_cpp)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
add_executable(edge_detector src/edge_detector.cpp)
target_compile_features(edge_detector PUBLIC cxx_std_17)
ament_target_dependencies(edge_detector rclcpp sensor_msgs)
install(TARGETS edge_detector DESTINATION lib/${PROJECT_NAME})
ament_package()
```

- **Interviewer's target sentence:** "Targets and usage requirements, not global flags:
  `target_link_libraries` with the right visibility, `RelWithDebInfo` for what ships,
  `compile_commands.json` for tooling; in ROS 2 that's an `ament_cmake` package colcon builds
  next to the Python ones."
- **`piros2` line:** the one true C++ contact. CLAUDE.md's convention reads "Python packages
  use `ament_python`; C++ uses `ament_cmake`"; `just build` runs `colcon build
  --symlink-install`, the Ansible `workspace` role runs the same on the Pi, and the repo's
  own docs note that a custom stats message "needs a rosidl `ament_cmake` package" — the
  reason counts ride `std_msgs/Int32`. A genuine linking lesson lives in
  `depth_estimator`: `onnxruntime.preload_dlls()` `dlopen`s the pip-installed CUDA/cuDNN
  libraries "where the system loader never looks" so the CUDA provider can link — a
  `LD_LIBRARY_PATH`/`RPATH` story told through Python.

## 19. Package management: Conan, vcpkg

- C++ has no `pip`; the ecosystem splits between the OS package manager (apt — pinned by
  distro, what ROS 2 uses), source vendoring (`FetchContent`, CPM, git submodules), and two
  real package managers. **Conan** (JFrog; Conan 2 since 2023): Python-based, `conanfile.py`/
  `.txt`, *profiles* (compiler, version, arch, build type, `libcxx`) select or build
  *binary* packages, remotes (ConanCenter), CMake integration via the `CMakeDeps` +
  `CMakeToolchain` generators (`conan install . && cmake --preset conan-release`); strong on
  cross-compilation and private binary caches — the embedded/enterprise choice. **vcpkg**
  (Microsoft): a ports tree, builds from source by default with a binary cache, *manifest
  mode* (`vcpkg.json` beside `CMakeLists.txt`), integrates through
  `-DCMAKE_TOOLCHAIN_FILE=…/vcpkg.cmake`, triplets (`x64-linux`, `arm64-linux`) for
  cross builds; simplest for a CMake project on a workstation.
- **The ROS 2 answer** is neither: dependencies are declared in `package.xml`
  (`<depend>rclcpp</depend>`), resolved by **rosdep** to apt/pip packages, published as
  `ros-jazzy-*` debs through bloom/the buildfarm, and workspace-overlaid by colcon —
  `vcs` (`.repos` files) pulls source dependencies. Docker images pin the whole set. Where
  Conan/vcpkg meet ROS in practice: third-party non-ROS libraries (a LiDAR SDK, a newer
  Eigen/Ceres) vendored inside a `_vendor` package or pulled by `FetchContent`.
- **Interviewer's target sentence:** "In a ROS 2 tree the package manager is rosdep plus
  apt with `package.xml` as the manifest; Conan is what I'd reach for when a product needs
  reproducible cross-compiled binaries of non-ROS deps, vcpkg when it's one CMake project on
  a workstation."
- **`piros2` line:** rosdep/apt exactly — the Ansible `ros2_install` role installs
  `ros-jazzy-*` metapackages plus `extra_ros_packages` from `group_vars`, `python3-rosdep`
  and `python3-vcstool`; the PyPI-only pieces (onnxruntime-gpu, open3d) live in a documented
  venv escape hatch. Conan and vcpkg: not touched.

## 20. Debugging: gdb, valgrind, sanitizers (ASan, TSan, UBSan)

- **gdb:** build `-g` (with `-O0` or `-Og` for a faithful view; `RelWithDebInfo` for
  field-like bugs), `break file:line`, `run`, `bt`, `frame N`, `p expr`, `watch var`
  (hardware watchpoints for "who changes this"), `catch throw`, `thread apply all bt`,
  `info threads`; core dumps via `ulimit -c unlimited` and `coredumpctl gdb`; attach with
  `gdb -p PID`. In ROS 2: `ros2 run --prefix 'gdb -ex run --args' pkg exe`, or `launch_ros`'s
  `prefix=['gdb -ex run --args']` on the `Node`; `gdbserver` on the robot, `gdb` on the
  laptop with the same sysroot. Pretty-printers make `std::` types readable.
- **valgrind memcheck:** no recompilation, ~20–50× slowdown, catches invalid reads/writes,
  uninitialised use, leaks (`--leak-check=full`); `helgrind`/`drd` for races; too slow for
  anything at frame rate. **Sanitizers** are compiled in (`-fsanitize=…`, clang or gcc, keep
  `-g` and `-fno-omit-frame-pointer`): **ASan** (~2× slower, ~3× memory; heap/stack/global
  overflow, use-after-free/return/scope, leaks via LSan — the daily driver), **TSan** (data
  races and lock-order inversions; 5–15× slower, 5–10× memory; cannot combine with ASan;
  every library on the path should be instrumented or you get false negatives), **UBSan**
  (near-zero cost; signed overflow, shifts, null, misalignment, vptr; use
  `-fno-sanitize-recover` to abort on the first hit), **MSan** (clang-only, uninitialised
  reads, needs an instrumented libc++). Run the unit tests under ASan+UBSan and TSan in CI
  as separate jobs; run a bag replay under TSan once per executor change.
- **Interviewer's target sentence:** "ASan and UBSan on every CI test run, TSan on the
  multithreaded ones, valgrind when I can't rebuild, gdb with a core dump for what the field
  sends back — and on the robot itself, `gdbserver` and a symbol file, not printf."
- **`piros2` line:** not touched — no C++ to sanitise. The repo's debugging is by evidence
  files: `just snap`, `gate_check.py`, and per-node logs; a Python traceback is the crash
  dump. Its "verify by script, not eyes" rule is the same instinct that puts sanitizers in
  CI.

## 21. Profiling: perf, flame graphs, cachegrind

- **`perf`** (Linux, hardware counters + sampling): `perf stat -e cycles,instructions,
  cache-misses,branch-misses ./node` for IPC and miss rates; `perf record -g` (needs frame
  pointers — build with `-fno-omit-frame-pointer` — or `--call-graph dwarf`) then `perf
  report`/`perf top`; `perf trace` for syscalls; works on ARM (Pi, Jetson) with the same
  commands, given `perf_event_paranoid` allows it. **Flame graphs** (Brendan Gregg:
  `perf script | stackcollapse-perf.pl | flamegraph.pl`): width = share of samples, y =
  stack depth, *not* time-ordered — the tool that finds the 30 % you didn't expect;
  `hotspot` is the GUI. **cachegrind/callgrind** (valgrind): simulated cache — instruction
  and data miss rates per line (`cg_annotate`), exact call counts (`callgrind` +
  KCachegrind); slow, deterministic, ideal for a unit-sized hot loop.
- The others in the kit: `heaptrack`/`massif` (allocation), `ltrace -e malloc` (does the
  loop allocate?), `strace -c` (syscall counts), Tracy/VTune, and for ROS 2 specifically
  `ros2_tracing` (LTTng instrumentation of rclcpp/rmw — callback durations, executor wake
  latency). Method: measure end-to-end first, one variable at a time, on the target
  hardware and its clock, and keep the numbers with the code.
- **Interviewer's target sentence:** "`perf stat` for the counters, `perf record -g` and a
  flame graph for where the time goes, cachegrind when I suspect the memory hierarchy — on
  the real ARM target, because the laptop's caches lie about the robot's."
- **`piros2` line:** the repo measures relentlessly but in Python, per node, against
  `time.monotonic()`: "~14 ms/frame ORB", "72–79 ms/frame depth on the GPU vs 280–305 ms
  CPU", "27–30 ms per 45k-point cloud", "52–78 ms/frame TSDF" — wall-clock deltas logged by
  the node, never `perf`. Those numbers are the discipline; `perf` and flame graphs are
  what the C++ port would add.

## 22. Compiler optimisation, inlining, LTO

- **Levels:** `-O0` (debuggable, slow), `-O1`, `-O2` (the production default —
  vectorisation is on at `-O2` since GCC 12 with the cheap cost model), `-O3` (aggressive
  unrolling/vectorising, sometimes slower on small caches), `-Os` (size — embedded), `-Og`
  (debug-friendly), `-Ofast` (breaks IEEE semantics — `-ffast-math`, don't for estimation
  code). `-march=native` / `-mcpu=cortex-a76` (Pi 5) unlock the target's SIMD (AVX2 vs NEON);
  `-DNDEBUG` strips `assert`. Always compare on the target hardware.
- **Inlining** is the enabling optimisation — once a call is inlined, constant propagation,
  dead-code elimination and vectorisation apply across it. The `inline` keyword is about the
  ODR (define in a header), not a command; the compiler decides by heuristics, and
  `[[gnu::always_inline]]`/`__attribute__((noinline))` are the overrides. Templates and
  lambdas inline naturally; `std::function` and virtual calls don't (devirtualisation with
  `final` helps).
- **LTO** (`-flto`; ThinLTO in clang): defer optimisation to link time so the compiler sees
  the whole program — cross-TU inlining, dead symbol removal — typically a few to 10 %
  faster and smaller binaries for a longer link. **PGO** (`-fprofile-generate` → run a
  representative bag → `-fprofile-use`) tells the optimiser which branches are hot. Also:
  `restrict`/`__restrict` to promise no aliasing (Eigen wants it), `-fno-exceptions
  -fno-rtti` on firmware, `[[likely]]`/`[[unlikely]]` (C++20). Compiler Explorer (godbolt) to
  *look* at what the compiler did.
- **Interviewer's target sentence:** "`-O2` with the target `-mcpu`, LTO on the release
  build, and I check the hot loop vectorised in godbolt or `perf` before I trust it; `-O3`
  and `-ffast-math` only with a benchmark on the real board."
- **`piros2` line:** not touched. The equivalent decision the repo did make is numpy
  vectorisation ("no loops" in `cloud_projector`) and moving inference to CUDA — the same
  instinct (let the compiled kernel own the inner loop) at a coarser grain. The C++ port
  would get its SIMD from `-mcpu`/Eigen instead.

## 23. Static analysis: clang-tidy, cppcheck

- **clang-tidy:** the clang-based linter/modernizer; needs `compile_commands.json`; check
  families `bugprone-*`, `performance-*` (unnecessary copies, `const&` params),
  `modernize-*` (use `nullptr`, `make_unique`, range-for), `readability-*`,
  `cppcoreguidelines-*`, `cert-*`, `concurrency-*`, `misc-*`; configured by a `.clang-tidy`
  file at the repo root; `-fix` applies safe rewrites; `run-clang-tidy` across the build. Its
  sibling **clang-format** is style only. **cppcheck:** independent of the compiler,
  works without a compile database, deliberately low false-positive rate ("unsound by
  design"), `--enable=all --project=compile_commands.json`, MISRA/CERT addons for
  safety-critical rule sets. Beyond those: `-Wall -Wextra -Wpedantic -Wshadow -Wconversion
  -Werror` (the cheapest static analysis there is), the clang static analyzer (`scan-build`),
  and commercial tools (Coverity, PVS-Studio, SonarQube) in safety/defence contexts.
- **In ROS 2:** the `ament_lint` family — `ament_cppcheck`, `ament_cpplint`,
  `ament_uncrustify`/`ament_clang_format`, `ament_clang_tidy`, plus `ament_flake8`,
  `ament_pep257`, `ament_copyright`, `ament_xmllint` — run as tests when a package's
  `CMakeLists.txt` has `if(BUILD_TESTING) find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() endif()`, so `colcon test` fails on lint. That is
  the ROS convention for "static analysis in CI".
- **Interviewer's target sentence:** "Warnings as errors first, clang-tidy with a checked-in
  config on the compile database, cppcheck as the second opinion, and in a ROS 2 package it
  all runs under `ament_lint_auto` so `colcon test` is the gate."
- **`piros2` line:** the Python half of exactly that machinery is real: every package
  carries `test_flake8.py`, `test_pep257.py` and `test_copyright.py` importing
  `ament_flake8.main` / `ament_pep257.main`, anchored on `__file__` so the VSCode Testing
  sidebar and `just test` agree; the suite is "style-clean" by that gate. The C++ siblings
  (`ament_cppcheck`, `ament_clang_tidy`) are untouched, as is `.clang-tidy`.

## 24. Python interop: pybind11

- **pybind11:** header-only C++11+ library for Python bindings. `PYBIND11_MODULE(name, m)`
  defines the module; `m.def("f", &f, "doc", py::arg("x") = 1)` binds functions;
  `py::class_<T>(m, "T").def(py::init<int>()).def("method", &T::method).def_readwrite("x",
  &T::x)` binds classes; `<pybind11/stl.h>` converts `vector`/`map`/`optional`/`variant`
  by *copy*; `<pybind11/numpy.h>` gives `py::array_t<float>` over the buffer protocol —
  **zero-copy** access to numpy memory (`.unchecked<2>()`, `.request()` for shape/strides),
  and `<pybind11/eigen.h>` maps Eigen matrices. Ownership: holder types (`std::shared_ptr<T>`
  as the class holder), return-value policies (`take_ownership`, `reference_internal`,
  `copy`, `move`, `automatic`) — get one wrong and Python frees C++ memory. **The GIL:** a
  bound function holds it by default; release with `py::call_guard<py::gil_scoped_release>()`
  for anything long or threaded, reacquire with `gil_scoped_acquire` before touching Python
  objects. Build with CMake's `pybind11_add_module` or scikit-build-core.
  Alternatives: nanobind (same author, faster/smaller, C++17), Boost.Python (heavy),
  SWIG/Cython/ctypes/cffi. The pattern that fits robotics: kernels and drivers in C++,
  bound once, orchestrated/tested from Python.
- **In ROS 2 this is not optional knowledge:** rclpy *is* a pybind11 module
  (`rclpy._rclpy_pybind11`) over the C library `rcl`; every Python node's publish, spin and
  parameter call crosses that boundary. Open3D's and onnxruntime's Python packages are
  pybind11 bindings; OpenCV generates its own. So a "Python ROS 2 stack" is a thin Python
  layer driving C/C++ through pybind11 all the way down.
- **Interviewer's target sentence:** "pybind11 for the boundary: bind the C++ kernel once,
  hand numpy buffers across zero-copy with `array_t`, release the GIL for the long calls,
  and be exact about return-value policies — rclpy itself is a pybind11 module over rcl,
  so I've been standing on it the whole time."
- **`piros2` line:** a *consumer* of pybind11 on every line — rclpy (the mesher even
  catches `rclpy._rclpy_pybind11.RCLError`), open3d (`VoxelBlockGrid.integrate`, whose
  `(float, float)`/`(uint16, uint8)` dtype-pair rule is a binding's type contract showing
  through) and onnxruntime (`preload_dlls()`, the CUDA provider). Never a *producer*: no
  `PYBIND11_MODULE` in the tree. The GIL story in `mesh_worker.py` — Open3D's decimation
  holds the GIL, so a Python thread couldn't help — is the `gil_scoped_release` lesson from
  the calling side.

## What to say if asked "how's your C++?"

"Honest answer: my C++ is embedded firmware C/C++ — bare-metal and RTOS-style code, fixed
buffers, no exceptions, no heap in the loop — not large-scale application C++ or the STL. I
know modern C++ at the reading-and-reasoning level: RAII and the rule of zero, `unique_ptr`
versus `shared_ptr`, move semantics, C++17's `optional`/`variant`/`string_view`, what C++20
concepts and `span` change, executors and callback groups in rclcpp, and how I'd build an
SPSC ring with acquire/release for a sensor thread. My ROS 2 project, `piros2`, is entirely
Python — six `ament_python` packages, ~200 tests, colcon-built beside where the `ament_cmake`
packages would go — and everything in it that has to be fast is C++ underneath through
pybind11: rclpy, Open3D, onnxruntime, OpenCV. The nodes I'd port first are the ones the
Python measurements already indict — the mesher's GIL-bound worker and the point-cloud
projector — and I'd port them as rclcpp components with a worker thread and a ring buffer.
That's the gap, that's the plan, and I won't claim the C++ until it's built." Then stop.
