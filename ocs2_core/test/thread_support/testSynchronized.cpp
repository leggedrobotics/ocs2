#include <gtest/gtest.h>

#include <ocs2_core/thread_support/Synchronized.h>

#include <functional>

enum class ClassId { A, B };

// Base class
class A {
 public:
  A() = default;
  virtual ~A() = default;
  A(const A&) = delete;
  A& operator=(const A&) = delete;
  virtual ClassId getId() const { return ClassId::A; }
  virtual int getVal() { return 0; }
  static void run(const std::function<void()>& f) { f(); }
};

// Derived class
class B : public A {
 public:
  B() = default;
  ~B() override = default;
  B(const B&) = delete;
  B& operator=(const B&) = delete;
  ClassId getId() const override { return ClassId::B; }
  int getVal() override { return 1; };
};

TEST(testLockable, construction) {
  // default
  ocs2::Synchronized<double> defaultObj;
  ASSERT_FALSE(defaultObj.lock());

  // from pointer
  auto doublePtr = std::make_unique<double>(1.0);
  ocs2::Synchronized<double> nonDefaultObj(std::move(doublePtr));
  ASSERT_TRUE(nonDefaultObj.lock());
}

TEST(testLockable, polymorphic) {
  std::unique_ptr<A> objB(new B());
  ocs2::Synchronized<A> synchronized(std::move(objB));

  // Direct access
  ASSERT_EQ(synchronized->getId(), ClassId::B);

  // Reset
  std::unique_ptr<A> objA(new A());
  synchronized.reset(std::move(objA));

  // Direct access
  ASSERT_EQ(synchronized->getId(), ClassId::A);
}

TEST(testLockable, lockedWhileDirectCalling) {
  std::unique_ptr<A> objB(new B());
  ocs2::Synchronized<A> synchronized(std::move(objB));

  // Direct call
  synchronized->run([&]() {
    // Check that the mutex is locked while inside A::run()
    ASSERT_FALSE(synchronized.getMutex().try_lock());
  });

  // Chained direct call
  synchronized.lock()->run([&]() {
    // Check that the mutex is locked while inside A::run()
    ASSERT_FALSE(synchronized.getMutex().try_lock());
  });
}

TEST(testLockable, scopedLocking) {
  std::unique_ptr<A> objB(new B());
  ocs2::Synchronized<A> synchronized(std::move(objB));

  {
    auto lockedPtr = synchronized.lock();
    ASSERT_TRUE(lockedPtr);
    ASSERT_FALSE(synchronized.getMutex().try_lock());
    ASSERT_EQ(lockedPtr->getId(), ClassId::B);
    ASSERT_EQ(lockedPtr->getVal(), 1);
  }

  // Mutex is released
  ASSERT_TRUE(synchronized.getMutex().try_lock());
  synchronized.getMutex().unlock();
}

TEST(testLockable, scopedLockingConst) {
  std::unique_ptr<A> objB(new B());
  ocs2::Synchronized<A> synchronized(std::move(objB));

  // Const ref, as if passed to a function
  const ocs2::Synchronized<A>& synchronizedRef = synchronized;

  {
    auto lockedConstPtr = synchronizedRef.lock();
    // Can only call const methods here
    ASSERT_TRUE(lockedConstPtr);
    ASSERT_FALSE(synchronized.getMutex().try_lock());
    ASSERT_EQ(lockedConstPtr->getId(), ClassId::B);
  }
}

TEST(testLockable, resetWhileScopedLocking) {
  std::unique_ptr<A> objB(new B());
  ocs2::Synchronized<A> synchronized(std::move(objB));

  // Direct access
  ASSERT_EQ(synchronized->getId(), ClassId::B);

  {
    auto lockedPtr = synchronized.lock();
    ASSERT_TRUE(lockedPtr);
    ASSERT_FALSE(synchronized.getMutex().try_lock());
    ASSERT_EQ(lockedPtr->getId(), ClassId::B);
    ASSERT_EQ(lockedPtr->getVal(), 1);

    // Reset
    std::unique_ptr<A> objA(new A());
    lockedPtr.reset(std::move(objA));

    ASSERT_TRUE(lockedPtr);
    ASSERT_FALSE(synchronized.getMutex().try_lock());
    ASSERT_EQ(lockedPtr->getId(), ClassId::A);
    ASSERT_EQ(lockedPtr->getVal(), 0);
  }

  // Direct access, double check
  ASSERT_EQ(synchronized->getId(), ClassId::A);
}

TEST(testLockable, swap) {
  std::unique_ptr<A> objBefore(new A());
  std::unique_ptr<A> objAfter(new B());

  // Construct with objBefore
  ocs2::Synchronized<A> synchronized(std::move(objBefore));
  ASSERT_EQ(synchronized->getId(), ClassId::A);

  // Swap objBefore for objAfter
  synchronized.swap(objAfter);

  // Check the swap
  ASSERT_EQ(synchronized->getId(), ClassId::B);
  ASSERT_EQ(objAfter->getId(), ClassId::A);
}

TEST(testLockable, lockMultiple) {
  // Mix of types, const, and ref
  ocs2::Synchronized<A> synchronizedA(std::unique_ptr<A>(new A()));
  const ocs2::Synchronized<B> synchronizedB(std::unique_ptr<B>(new B()));
  ocs2::Synchronized<double> synchronizedDouble(std::unique_ptr<double>(new double(0.0)));
  auto& synchronizedDoubleRef = synchronizedDouble;

  {
    auto lockedPtrTuple = synchronizeLock(synchronizedA, synchronizedB, synchronizedDoubleRef);
    // Check that the returned lockedPtr is of the same type as if a regular lock was called
    ASSERT_EQ(typeid(std::get<0>(lockedPtrTuple)), typeid(decltype(synchronizedA.lock())));
    ASSERT_EQ(typeid(std::get<1>(lockedPtrTuple)), typeid(decltype(synchronizedB.lock())));
    ASSERT_EQ(typeid(std::get<2>(lockedPtrTuple)), typeid(decltype(synchronizedDoubleRef.lock())));

    // Check that all mutexes are locked
    ASSERT_FALSE(synchronizedA.getMutex().try_lock());
    ASSERT_FALSE(synchronizedB.getMutex().try_lock());
    ASSERT_FALSE(synchronizedDouble.getMutex().try_lock());
  }

  // Check that all mutexes are released
  ASSERT_TRUE(synchronizedA.getMutex().try_lock());
  ASSERT_TRUE(synchronizedB.getMutex().try_lock());
  ASSERT_TRUE(synchronizedDouble.getMutex().try_lock());
  synchronizedA.getMutex().unlock();
  synchronizedB.getMutex().unlock();
  synchronizedDouble.getMutex().unlock();
}
