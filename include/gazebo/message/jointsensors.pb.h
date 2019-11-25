// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: jointsensors.proto

#ifndef PROTOBUF_jointsensors_2eproto__INCLUDED
#define PROTOBUF_jointsensors_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>
#include <ignition/math.hh>
#if GOOGLE_PROTOBUF_VERSION < 3000000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3000000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

namespace QRsensor {
namespace msgs {

// Internal implementation detail -- do not call these.
void protobuf_AddDesc_jointsensors_2eproto();
void protobuf_AssignDesc_jointsensors_2eproto();
void protobuf_ShutdownFile_jointsensors_2eproto();

class AllJointSensors;
class ForceTorque;
class Time;
class Vector3d;
class Wrench;

// ===================================================================

class Time : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:QRsensor.msgs.Time) */ {
 public:
  Time();
  virtual ~Time();

  Time(const Time& from);

  inline Time& operator=(const Time& from) {
    CopyFrom(from);
    return *this;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const Time& default_instance();

  void Swap(Time* other);

  // implements Message ----------------------------------------------

  inline Time* New() const { return New(NULL); }

  Time* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Time& from);
  void MergeFrom(const Time& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const {
    return InternalSerializeWithCachedSizesToArray(false, output);
  }
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  void InternalSwap(Time* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return _internal_metadata_.arena();
  }
  inline void* MaybeArenaPtr() const {
    return _internal_metadata_.raw_arena_ptr();
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional int32 sec = 1;
  void clear_sec();
  static const int kSecFieldNumber = 1;
  ::google::protobuf::int32 sec() const;
  void set_sec(::google::protobuf::int32 value);

  // optional int32 nsec = 2;
  void clear_nsec();
  static const int kNsecFieldNumber = 2;
  ::google::protobuf::int32 nsec() const;
  void set_nsec(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:QRsensor.msgs.Time)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  bool _is_default_instance_;
  ::google::protobuf::int32 sec_;
  ::google::protobuf::int32 nsec_;
  mutable int _cached_size_;
  friend void  protobuf_AddDesc_jointsensors_2eproto();
  friend void protobuf_AssignDesc_jointsensors_2eproto();
  friend void protobuf_ShutdownFile_jointsensors_2eproto();

  void InitAsDefaultInstance();
  static Time* default_instance_;
};
// -------------------------------------------------------------------

class Vector3d : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:QRsensor.msgs.Vector3d) */ {
 public:
  Vector3d();
  virtual ~Vector3d();

  Vector3d(const Vector3d& from);

  inline Vector3d& operator=(const Vector3d& from) {
    CopyFrom(from);
    return *this;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const Vector3d& default_instance();

  void Swap(Vector3d* other);

  // implements Message ----------------------------------------------

  inline Vector3d* New() const { return New(NULL); }

  Vector3d* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Vector3d& from);
  void MergeFrom(const Vector3d& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const {
    return InternalSerializeWithCachedSizesToArray(false, output);
  }
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  void InternalSwap(Vector3d* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return _internal_metadata_.arena();
  }
  inline void* MaybeArenaPtr() const {
    return _internal_metadata_.raw_arena_ptr();
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional double x = 1;
  void clear_x();
  static const int kXFieldNumber = 1;
  double x() const;
  void set_x(double value);

  // optional double y = 2;
  void clear_y();
  static const int kYFieldNumber = 2;
  double y() const;
  void set_y(double value);

  // optional double z = 3;
  void clear_z();
  static const int kZFieldNumber = 3;
  double z() const;
  void set_z(double value);

  // @@protoc_insertion_point(class_scope:QRsensor.msgs.Vector3d)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  bool _is_default_instance_;
  double x_;
  double y_;
  double z_;
  mutable int _cached_size_;
  friend void  protobuf_AddDesc_jointsensors_2eproto();
  friend void protobuf_AssignDesc_jointsensors_2eproto();
  friend void protobuf_ShutdownFile_jointsensors_2eproto();

  void InitAsDefaultInstance();
  static Vector3d* default_instance_;
};
// -------------------------------------------------------------------

class Wrench : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:QRsensor.msgs.Wrench) */ {
 public:
  Wrench();
  virtual ~Wrench();

  Wrench(const Wrench& from);

  inline Wrench& operator=(const Wrench& from) {
    CopyFrom(from);
    return *this;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const Wrench& default_instance();

  void Swap(Wrench* other);

  // implements Message ----------------------------------------------

  inline Wrench* New() const { return New(NULL); }

  Wrench* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Wrench& from);
  void MergeFrom(const Wrench& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const {
    return InternalSerializeWithCachedSizesToArray(false, output);
  }
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  void InternalSwap(Wrench* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return _internal_metadata_.arena();
  }
  inline void* MaybeArenaPtr() const {
    return _internal_metadata_.raw_arena_ptr();
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional .QRsensor.msgs.Vector3d force = 1;
  bool has_force() const;
  void clear_force();
  static const int kForceFieldNumber = 1;
  const ::QRsensor::msgs::Vector3d& force() const;
  ::QRsensor::msgs::Vector3d* mutable_force();
  ::QRsensor::msgs::Vector3d* release_force();
  void set_allocated_force(::QRsensor::msgs::Vector3d* force);

  // optional .QRsensor.msgs.Vector3d torque = 2;
  bool has_torque() const;
  void clear_torque();
  static const int kTorqueFieldNumber = 2;
  const ::QRsensor::msgs::Vector3d& torque() const;
  ::QRsensor::msgs::Vector3d* mutable_torque();
  ::QRsensor::msgs::Vector3d* release_torque();
  void set_allocated_torque(::QRsensor::msgs::Vector3d* torque);

  // optional .QRsensor.msgs.Vector3d force_offset = 3;
  bool has_force_offset() const;
  void clear_force_offset();
  static const int kForceOffsetFieldNumber = 3;
  const ::QRsensor::msgs::Vector3d& force_offset() const;
  ::QRsensor::msgs::Vector3d* mutable_force_offset();
  ::QRsensor::msgs::Vector3d* release_force_offset();
  void set_allocated_force_offset(::QRsensor::msgs::Vector3d* force_offset);

  // @@protoc_insertion_point(class_scope:QRsensor.msgs.Wrench)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  bool _is_default_instance_;
  ::QRsensor::msgs::Vector3d* force_;
  ::QRsensor::msgs::Vector3d* torque_;
  ::QRsensor::msgs::Vector3d* force_offset_;
  mutable int _cached_size_;
  friend void  protobuf_AddDesc_jointsensors_2eproto();
  friend void protobuf_AssignDesc_jointsensors_2eproto();
  friend void protobuf_ShutdownFile_jointsensors_2eproto();

  void InitAsDefaultInstance();
  static Wrench* default_instance_;
};
// -------------------------------------------------------------------

class ForceTorque : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:QRsensor.msgs.ForceTorque) */ {
 public:
  ForceTorque();
  virtual ~ForceTorque();

  ForceTorque(const ForceTorque& from);

  inline ForceTorque& operator=(const ForceTorque& from) {
    CopyFrom(from);
    return *this;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const ForceTorque& default_instance();

  void Swap(ForceTorque* other);

  // implements Message ----------------------------------------------

  inline ForceTorque* New() const { return New(NULL); }

  ForceTorque* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const ForceTorque& from);
  void MergeFrom(const ForceTorque& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const {
    return InternalSerializeWithCachedSizesToArray(false, output);
  }
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  void InternalSwap(ForceTorque* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return _internal_metadata_.arena();
  }
  inline void* MaybeArenaPtr() const {
    return _internal_metadata_.raw_arena_ptr();
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional .QRsensor.msgs.Wrench wrench = 1;
  bool has_wrench() const;
  void clear_wrench();
  static const int kWrenchFieldNumber = 1;
  const ::QRsensor::msgs::Wrench& wrench() const;
  ::QRsensor::msgs::Wrench* mutable_wrench();
  ::QRsensor::msgs::Wrench* release_wrench();
  void set_allocated_wrench(::QRsensor::msgs::Wrench* wrench);

  // optional .QRsensor.msgs.Time time = 2;
  bool has_time() const;
  void clear_time();
  static const int kTimeFieldNumber = 2;
  const ::QRsensor::msgs::Time& time() const;
  ::QRsensor::msgs::Time* mutable_time();
  ::QRsensor::msgs::Time* release_time();
  void set_allocated_time(::QRsensor::msgs::Time* time);

  // @@protoc_insertion_point(class_scope:QRsensor.msgs.ForceTorque)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  bool _is_default_instance_;
  ::QRsensor::msgs::Wrench* wrench_;
  ::QRsensor::msgs::Time* time_;
  mutable int _cached_size_;
  friend void  protobuf_AddDesc_jointsensors_2eproto();
  friend void protobuf_AssignDesc_jointsensors_2eproto();
  friend void protobuf_ShutdownFile_jointsensors_2eproto();

  void InitAsDefaultInstance();
  static ForceTorque* default_instance_;
};
// -------------------------------------------------------------------

class AllJointSensors : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:QRsensor.msgs.AllJointSensors) */ {
 public:
  AllJointSensors();
  virtual ~AllJointSensors();

  AllJointSensors(const AllJointSensors& from);

  inline AllJointSensors& operator=(const AllJointSensors& from) {
    CopyFrom(from);
    return *this;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const AllJointSensors& default_instance();

  void Swap(AllJointSensors* other);

  // implements Message ----------------------------------------------

  inline AllJointSensors* New() const { return New(NULL); }

  AllJointSensors* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const AllJointSensors& from);
  void MergeFrom(const AllJointSensors& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const {
    return InternalSerializeWithCachedSizesToArray(false, output);
  }
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  void InternalSwap(AllJointSensors* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return _internal_metadata_.arena();
  }
  inline void* MaybeArenaPtr() const {
    return _internal_metadata_.raw_arena_ptr();
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // repeated .QRsensor.msgs.ForceTorque jointwrench = 1;
  int jointwrench_size() const;
  void clear_jointwrench();
  static const int kJointwrenchFieldNumber = 1;
  const ::QRsensor::msgs::ForceTorque& jointwrench(int index) const;
  ::QRsensor::msgs::ForceTorque* mutable_jointwrench(int index);
  ::QRsensor::msgs::ForceTorque* add_jointwrench();
  ::google::protobuf::RepeatedPtrField< ::QRsensor::msgs::ForceTorque >*
      mutable_jointwrench();
  const ::google::protobuf::RepeatedPtrField< ::QRsensor::msgs::ForceTorque >&
      jointwrench() const;

  // @@protoc_insertion_point(class_scope:QRsensor.msgs.AllJointSensors)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  bool _is_default_instance_;
  ::google::protobuf::RepeatedPtrField< ::QRsensor::msgs::ForceTorque > jointwrench_;
  mutable int _cached_size_;
  friend void  protobuf_AddDesc_jointsensors_2eproto();
  friend void protobuf_AssignDesc_jointsensors_2eproto();
  friend void protobuf_ShutdownFile_jointsensors_2eproto();

  void InitAsDefaultInstance();
  static AllJointSensors* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// Time

// optional int32 sec = 1;
inline void Time::clear_sec() {
  sec_ = 0;
}
inline ::google::protobuf::int32 Time::sec() const {
  // @@protoc_insertion_point(field_get:QRsensor.msgs.Time.sec)
  return sec_;
}
inline void Time::set_sec(::google::protobuf::int32 value) {
  
  sec_ = value;
  // @@protoc_insertion_point(field_set:QRsensor.msgs.Time.sec)
}

// optional int32 nsec = 2;
inline void Time::clear_nsec() {
  nsec_ = 0;
}
inline ::google::protobuf::int32 Time::nsec() const {
  // @@protoc_insertion_point(field_get:QRsensor.msgs.Time.nsec)
  return nsec_;
}
inline void Time::set_nsec(::google::protobuf::int32 value) {
  
  nsec_ = value;
  // @@protoc_insertion_point(field_set:QRsensor.msgs.Time.nsec)
}

// -------------------------------------------------------------------

// Vector3d

// optional double x = 1;
inline void Vector3d::clear_x() {
  x_ = 0;
}
inline double Vector3d::x() const {
  // @@protoc_insertion_point(field_get:QRsensor.msgs.Vector3d.x)
  return x_;
}
inline void Vector3d::set_x(double value) {
  
  x_ = value;
  // @@protoc_insertion_point(field_set:QRsensor.msgs.Vector3d.x)
}

// optional double y = 2;
inline void Vector3d::clear_y() {
  y_ = 0;
}
inline double Vector3d::y() const {
  // @@protoc_insertion_point(field_get:QRsensor.msgs.Vector3d.y)
  return y_;
}
inline void Vector3d::set_y(double value) {
  
  y_ = value;
  // @@protoc_insertion_point(field_set:QRsensor.msgs.Vector3d.y)
}

// optional double z = 3;
inline void Vector3d::clear_z() {
  z_ = 0;
}
inline double Vector3d::z() const {
  // @@protoc_insertion_point(field_get:QRsensor.msgs.Vector3d.z)
  return z_;
}
inline void Vector3d::set_z(double value) {
  
  z_ = value;
  // @@protoc_insertion_point(field_set:QRsensor.msgs.Vector3d.z)
}

// -------------------------------------------------------------------

// Wrench

// optional .QRsensor.msgs.Vector3d force = 1;
inline bool Wrench::has_force() const {
  return !_is_default_instance_ && force_ != NULL;
}
inline void Wrench::clear_force() {
  if (GetArenaNoVirtual() == NULL && force_ != NULL) delete force_;
  force_ = NULL;
}
inline const ::QRsensor::msgs::Vector3d& Wrench::force() const {
  // @@protoc_insertion_point(field_get:QRsensor.msgs.Wrench.force)
  return force_ != NULL ? *force_ : *default_instance_->force_;
}
inline ::QRsensor::msgs::Vector3d* Wrench::mutable_force() {
  
  if (force_ == NULL) {
    force_ = new ::QRsensor::msgs::Vector3d;
  }
  // @@protoc_insertion_point(field_mutable:QRsensor.msgs.Wrench.force)
  return force_;
}
inline ::QRsensor::msgs::Vector3d* Wrench::release_force() {
  // @@protoc_insertion_point(field_release:QRsensor.msgs.Wrench.force)
  
  ::QRsensor::msgs::Vector3d* temp = force_;
  force_ = NULL;
  return temp;
}
inline void Wrench::set_allocated_force(::QRsensor::msgs::Vector3d* force) {
  delete force_;
  force_ = force;
  if (force) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_set_allocated:QRsensor.msgs.Wrench.force)
}

// optional .QRsensor.msgs.Vector3d torque = 2;
inline bool Wrench::has_torque() const {
  return !_is_default_instance_ && torque_ != NULL;
}
inline void Wrench::clear_torque() {
  if (GetArenaNoVirtual() == NULL && torque_ != NULL) delete torque_;
  torque_ = NULL;
}
inline const ::QRsensor::msgs::Vector3d& Wrench::torque() const {
  // @@protoc_insertion_point(field_get:QRsensor.msgs.Wrench.torque)
  return torque_ != NULL ? *torque_ : *default_instance_->torque_;
}
inline ::QRsensor::msgs::Vector3d* Wrench::mutable_torque() {
  
  if (torque_ == NULL) {
    torque_ = new ::QRsensor::msgs::Vector3d;
  }
  // @@protoc_insertion_point(field_mutable:QRsensor.msgs.Wrench.torque)
  return torque_;
}
inline ::QRsensor::msgs::Vector3d* Wrench::release_torque() {
  // @@protoc_insertion_point(field_release:QRsensor.msgs.Wrench.torque)
  
  ::QRsensor::msgs::Vector3d* temp = torque_;
  torque_ = NULL;
  return temp;
}
inline void Wrench::set_allocated_torque(::QRsensor::msgs::Vector3d* torque) {
  delete torque_;
  torque_ = torque;
  if (torque) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_set_allocated:QRsensor.msgs.Wrench.torque)
}

// optional .QRsensor.msgs.Vector3d force_offset = 3;
inline bool Wrench::has_force_offset() const {
  return !_is_default_instance_ && force_offset_ != NULL;
}
inline void Wrench::clear_force_offset() {
  if (GetArenaNoVirtual() == NULL && force_offset_ != NULL) delete force_offset_;
  force_offset_ = NULL;
}
inline const ::QRsensor::msgs::Vector3d& Wrench::force_offset() const {
  // @@protoc_insertion_point(field_get:QRsensor.msgs.Wrench.force_offset)
  return force_offset_ != NULL ? *force_offset_ : *default_instance_->force_offset_;
}
inline ::QRsensor::msgs::Vector3d* Wrench::mutable_force_offset() {
  
  if (force_offset_ == NULL) {
    force_offset_ = new ::QRsensor::msgs::Vector3d;
  }
  // @@protoc_insertion_point(field_mutable:QRsensor.msgs.Wrench.force_offset)
  return force_offset_;
}
inline ::QRsensor::msgs::Vector3d* Wrench::release_force_offset() {
  // @@protoc_insertion_point(field_release:QRsensor.msgs.Wrench.force_offset)
  
  ::QRsensor::msgs::Vector3d* temp = force_offset_;
  force_offset_ = NULL;
  return temp;
}
inline void Wrench::set_allocated_force_offset(::QRsensor::msgs::Vector3d* force_offset) {
  delete force_offset_;
  force_offset_ = force_offset;
  if (force_offset) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_set_allocated:QRsensor.msgs.Wrench.force_offset)
}

// -------------------------------------------------------------------

// ForceTorque

// optional .QRsensor.msgs.Wrench wrench = 1;
inline bool ForceTorque::has_wrench() const {
  return !_is_default_instance_ && wrench_ != NULL;
}
inline void ForceTorque::clear_wrench() {
  if (GetArenaNoVirtual() == NULL && wrench_ != NULL) delete wrench_;
  wrench_ = NULL;
}
inline const ::QRsensor::msgs::Wrench& ForceTorque::wrench() const {
  // @@protoc_insertion_point(field_get:QRsensor.msgs.ForceTorque.wrench)
  return wrench_ != NULL ? *wrench_ : *default_instance_->wrench_;
}
inline ::QRsensor::msgs::Wrench* ForceTorque::mutable_wrench() {
  
  if (wrench_ == NULL) {
    wrench_ = new ::QRsensor::msgs::Wrench;
  }
  // @@protoc_insertion_point(field_mutable:QRsensor.msgs.ForceTorque.wrench)
  return wrench_;
}
inline ::QRsensor::msgs::Wrench* ForceTorque::release_wrench() {
  // @@protoc_insertion_point(field_release:QRsensor.msgs.ForceTorque.wrench)
  
  ::QRsensor::msgs::Wrench* temp = wrench_;
  wrench_ = NULL;
  return temp;
}
inline void ForceTorque::set_allocated_wrench(::QRsensor::msgs::Wrench* wrench) {
  delete wrench_;
  wrench_ = wrench;
  if (wrench) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_set_allocated:QRsensor.msgs.ForceTorque.wrench)
}

// optional .QRsensor.msgs.Time time = 2;
inline bool ForceTorque::has_time() const {
  return !_is_default_instance_ && time_ != NULL;
}
inline void ForceTorque::clear_time() {
  if (GetArenaNoVirtual() == NULL && time_ != NULL) delete time_;
  time_ = NULL;
}
inline const ::QRsensor::msgs::Time& ForceTorque::time() const {
  // @@protoc_insertion_point(field_get:QRsensor.msgs.ForceTorque.time)
  return time_ != NULL ? *time_ : *default_instance_->time_;
}
inline ::QRsensor::msgs::Time* ForceTorque::mutable_time() {
  
  if (time_ == NULL) {
    time_ = new ::QRsensor::msgs::Time;
  }
  // @@protoc_insertion_point(field_mutable:QRsensor.msgs.ForceTorque.time)
  return time_;
}
inline ::QRsensor::msgs::Time* ForceTorque::release_time() {
  // @@protoc_insertion_point(field_release:QRsensor.msgs.ForceTorque.time)
  
  ::QRsensor::msgs::Time* temp = time_;
  time_ = NULL;
  return temp;
}
inline void ForceTorque::set_allocated_time(::QRsensor::msgs::Time* time) {
  delete time_;
  time_ = time;
  if (time) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_set_allocated:QRsensor.msgs.ForceTorque.time)
}

// -------------------------------------------------------------------

// AllJointSensors

// repeated .QRsensor.msgs.ForceTorque jointwrench = 1;
inline int AllJointSensors::jointwrench_size() const {
  return jointwrench_.size();
}
inline void AllJointSensors::clear_jointwrench() {
  jointwrench_.Clear();
}
inline const ::QRsensor::msgs::ForceTorque& AllJointSensors::jointwrench(int index) const {
  // @@protoc_insertion_point(field_get:QRsensor.msgs.AllJointSensors.jointwrench)
  return jointwrench_.Get(index);
}
inline ::QRsensor::msgs::ForceTorque* AllJointSensors::mutable_jointwrench(int index) {
  // @@protoc_insertion_point(field_mutable:QRsensor.msgs.AllJointSensors.jointwrench)
  return jointwrench_.Mutable(index);
}
inline ::QRsensor::msgs::ForceTorque* AllJointSensors::add_jointwrench() {
  // @@protoc_insertion_point(field_add:QRsensor.msgs.AllJointSensors.jointwrench)
  return jointwrench_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::QRsensor::msgs::ForceTorque >*
AllJointSensors::mutable_jointwrench() {
  // @@protoc_insertion_point(field_mutable_list:QRsensor.msgs.AllJointSensors.jointwrench)
  return &jointwrench_;
}
inline const ::google::protobuf::RepeatedPtrField< ::QRsensor::msgs::ForceTorque >&
AllJointSensors::jointwrench() const {
  // @@protoc_insertion_point(field_list:QRsensor.msgs.AllJointSensors.jointwrench)
  return jointwrench_;
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS
// -------------------------------------------------------------------

// -------------------------------------------------------------------

// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace QRsensor

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_jointsensors_2eproto__INCLUDED
