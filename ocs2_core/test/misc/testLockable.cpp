//
// Created by rgrandia on 24.08.20.
//

#include <gtest/gtest.h>

#include <ocs2_core/misc/Lockable.h>

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
  void run(const std::function<void()>& f) { f(); }
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

TEST(testLockable, defaultConstruction) {
  // default
  ocs2::Synchronized<double> defaultObj;
  ASSERT_FALSE(defaultObj.lock());

  // from pointer
  std::unique_ptr<double> doublePtr(new double(1.0));
  ocs2::Synchronized<double> nonDefaultObj(std::move(doublePtr));
  ASSERT_TRUE(nonDefaultObj.lock());
}

TEST(testLockable, polymorphic) {
  // from pointer
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
  // from pointer
  std::unique_ptr<A> objB(new B());
  ocs2::Synchronized<A> synchronized(std::move(objB));

  // Direct call
  synchronized->run([&](){
    // Check that the mutex is locked while inside A::run()
    ASSERT_FALSE(synchronized.getMutex().try_lock());
  });

  // Chained direct call
  synchronized.lock()->run([&](){
    // Check that the mutex is locked while inside A::run()
    ASSERT_FALSE(synchronized.getMutex().try_lock());
  });
}

TEST(testLockable, synchronize) {
  // from pointer
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

TEST(testLockable, synchronizeConst) {
  // from pointer
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

TEST(testLockable, resetWhileSynchronized) {
  // from pointer
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
    lockedPtr.reset(std::unique_ptr<A>(new A()));

    ASSERT_TRUE(lockedPtr);
    ASSERT_FALSE(synchronized.getMutex().try_lock());
    ASSERT_EQ(lockedPtr->getId(), ClassId::A);
    ASSERT_EQ(lockedPtr->getVal(), 0);
  }

  // Direct access, double check
  ASSERT_EQ(synchronized->getId(), ClassId::A);
}