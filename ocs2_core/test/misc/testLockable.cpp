//
// Created by rgrandia on 24.08.20.
//

#include <gtest/gtest.h>

#include <ocs2_core/misc/Lockable.h>

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
  ocs2::SynchronizedPtr<double> defaultObj;
  ASSERT_FALSE(defaultObj);

  // from pointer
  std::unique_ptr<double> doublePtr(new double(1.0));
  ocs2::SynchronizedPtr<double> nonDefaultObj(std::move(doublePtr));
  ASSERT_TRUE(nonDefaultObj);
}

TEST(testLockable, polymorphic) {
  // from pointer
  std::unique_ptr<A> objB(new B());
  ocs2::SynchronizedPtr<A> synchronizedPtr(std::move(objB));

  // Direct access
  ASSERT_EQ(synchronizedPtr->getId(), ClassId::B);

  // Reset
  std::unique_ptr<A> objA(new A());
  synchronizedPtr.reset(std::move(objA));

  // Direct access
  ASSERT_EQ(synchronizedPtr->getId(), ClassId::A);
}

TEST(testLockable, synchronize) {
  // from pointer
  std::unique_ptr<A> objB(new B());
  ocs2::SynchronizedPtr<A> synchronizedPtr(std::move(objB));

  {
    auto lockedPtr = synchronizedPtr.synchronize();
    ASSERT_TRUE(lockedPtr);
    ASSERT_EQ(lockedPtr->getId(), ClassId::B);
    ASSERT_EQ(lockedPtr->getVal(), 1);
  }
}

TEST(testLockable, synchronizeConst) {
  // from pointer
  std::unique_ptr<A> objB(new B());
  ocs2::SynchronizedPtr<A> synchronizedPtr(std::move(objB));

  // Const ref, as if passed to a function
  const ocs2::SynchronizedPtr<A>& synchronizedPtrRef = synchronizedPtr;

  {
    auto lockedPtr = synchronizedPtrRef.synchronize();
    // Can only call const methods here
    ASSERT_TRUE(lockedPtr);
    ASSERT_EQ(lockedPtr->getId(), ClassId::B);
  }
}

TEST(testLockable, resetWhileSynchronized) {
  // from pointer
  std::unique_ptr<A> objB(new B());
  ocs2::SynchronizedPtr<A> synchronizedPtr(std::move(objB));

  // Direct access
  ASSERT_EQ(synchronizedPtr->getId(), ClassId::B);

  {
    auto lockedPtr = synchronizedPtr.synchronize();
    ASSERT_TRUE(lockedPtr);
    ASSERT_EQ(lockedPtr->getId(), ClassId::B);
    ASSERT_EQ(lockedPtr->getVal(), 1);

    // Reset
    std::unique_ptr<A> objA(new A());
    lockedPtr.reset(std::move(objA));

    ASSERT_TRUE(lockedPtr);
    ASSERT_EQ(lockedPtr->getId(), ClassId::A);
    ASSERT_EQ(lockedPtr->getVal(), 0);
  }

  // Direct access, double check
  ASSERT_EQ(synchronizedPtr->getId(), ClassId::A);
}