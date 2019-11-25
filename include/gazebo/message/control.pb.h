// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: control.proto

#ifndef PROTOBUF_control_2eproto__INCLUDED
#define PROTOBUF_control_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

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

namespace QRcommand {
namespace msgs {

// Internal implementation detail -- do not call these.
void protobuf_AddDesc_control_2eproto();
void protobuf_AssignDesc_control_2eproto();
void protobuf_ShutdownFile_control_2eproto();

class Control;

// ===================================================================

class Control : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:QRcommand.msgs.Control) */ {
 public:
  Control();
  virtual ~Control();

  Control(const Control& from);

  inline Control& operator=(const Control& from) {
    CopyFrom(from);
    return *this;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const Control& default_instance();

  void Swap(Control* other);

  // implements Message ----------------------------------------------

  inline Control* New() const { return New(NULL); }

  Control* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Control& from);
  void MergeFrom(const Control& from);
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
  void InternalSwap(Control* other);
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

  // repeated int32 mode = 1;
  int mode_size() const;
  void clear_mode();
  static const int kModeFieldNumber = 1;
  ::google::protobuf::int32 mode(int index) const;
  void set_mode(int index, ::google::protobuf::int32 value);
  void add_mode(::google::protobuf::int32 value);
  const ::google::protobuf::RepeatedField< ::google::protobuf::int32 >&
      mode() const;
  ::google::protobuf::RepeatedField< ::google::protobuf::int32 >*
      mutable_mode();

  // repeated double position = 2;
  int position_size() const;
  void clear_position();
  static const int kPositionFieldNumber = 2;
  double position(int index) const;
  void set_position(int index, double value);
  void add_position(double value);
  const ::google::protobuf::RepeatedField< double >&
      position() const;
  ::google::protobuf::RepeatedField< double >*
      mutable_position();

  // repeated double torque = 3;
  int torque_size() const;
  void clear_torque();
  static const int kTorqueFieldNumber = 3;
  double torque(int index) const;
  void set_torque(int index, double value);
  void add_torque(double value);
  const ::google::protobuf::RepeatedField< double >&
      torque() const;
  ::google::protobuf::RepeatedField< double >*
      mutable_torque();

  // optional int32 type = 4;
  void clear_type();
  static const int kTypeFieldNumber = 4;
  ::google::protobuf::int32 type() const;
  void set_type(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:QRcommand.msgs.Control)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  bool _is_default_instance_;
  ::google::protobuf::RepeatedField< ::google::protobuf::int32 > mode_;
  mutable int _mode_cached_byte_size_;
  ::google::protobuf::RepeatedField< double > position_;
  mutable int _position_cached_byte_size_;
  ::google::protobuf::RepeatedField< double > torque_;
  mutable int _torque_cached_byte_size_;
  ::google::protobuf::int32 type_;
  mutable int _cached_size_;
  friend void  protobuf_AddDesc_control_2eproto();
  friend void protobuf_AssignDesc_control_2eproto();
  friend void protobuf_ShutdownFile_control_2eproto();

  void InitAsDefaultInstance();
  static Control* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// Control

// repeated int32 mode = 1;
inline int Control::mode_size() const {
  return mode_.size();
}
inline void Control::clear_mode() {
  mode_.Clear();
}
inline ::google::protobuf::int32 Control::mode(int index) const {
  // @@protoc_insertion_point(field_get:QRcommand.msgs.Control.mode)
  return mode_.Get(index);
}
inline void Control::set_mode(int index, ::google::protobuf::int32 value) {
  mode_.Set(index, value);
  // @@protoc_insertion_point(field_set:QRcommand.msgs.Control.mode)
}
inline void Control::add_mode(::google::protobuf::int32 value) {
  mode_.Add(value);
  // @@protoc_insertion_point(field_add:QRcommand.msgs.Control.mode)
}
inline const ::google::protobuf::RepeatedField< ::google::protobuf::int32 >&
Control::mode() const {
  // @@protoc_insertion_point(field_list:QRcommand.msgs.Control.mode)
  return mode_;
}
inline ::google::protobuf::RepeatedField< ::google::protobuf::int32 >*
Control::mutable_mode() {
  // @@protoc_insertion_point(field_mutable_list:QRcommand.msgs.Control.mode)
  return &mode_;
}

// repeated double position = 2;
inline int Control::position_size() const {
  return position_.size();
}
inline void Control::clear_position() {
  position_.Clear();
}
inline double Control::position(int index) const {
  // @@protoc_insertion_point(field_get:QRcommand.msgs.Control.position)
  return position_.Get(index);
}
inline void Control::set_position(int index, double value) {
  position_.Set(index, value);
  // @@protoc_insertion_point(field_set:QRcommand.msgs.Control.position)
}
inline void Control::add_position(double value) {
  position_.Add(value);
  // @@protoc_insertion_point(field_add:QRcommand.msgs.Control.position)
}
inline const ::google::protobuf::RepeatedField< double >&
Control::position() const {
  // @@protoc_insertion_point(field_list:QRcommand.msgs.Control.position)
  return position_;
}
inline ::google::protobuf::RepeatedField< double >*
Control::mutable_position() {
  // @@protoc_insertion_point(field_mutable_list:QRcommand.msgs.Control.position)
  return &position_;
}

// repeated double torque = 3;
inline int Control::torque_size() const {
  return torque_.size();
}
inline void Control::clear_torque() {
  torque_.Clear();
}
inline double Control::torque(int index) const {
  // @@protoc_insertion_point(field_get:QRcommand.msgs.Control.torque)
  return torque_.Get(index);
}
inline void Control::set_torque(int index, double value) {
  torque_.Set(index, value);
  // @@protoc_insertion_point(field_set:QRcommand.msgs.Control.torque)
}
inline void Control::add_torque(double value) {
  torque_.Add(value);
  // @@protoc_insertion_point(field_add:QRcommand.msgs.Control.torque)
}
inline const ::google::protobuf::RepeatedField< double >&
Control::torque() const {
  // @@protoc_insertion_point(field_list:QRcommand.msgs.Control.torque)
  return torque_;
}
inline ::google::protobuf::RepeatedField< double >*
Control::mutable_torque() {
  // @@protoc_insertion_point(field_mutable_list:QRcommand.msgs.Control.torque)
  return &torque_;
}

// optional int32 type = 4;
inline void Control::clear_type() {
  type_ = 0;
}
inline ::google::protobuf::int32 Control::type() const {
  // @@protoc_insertion_point(field_get:QRcommand.msgs.Control.type)
  return type_;
}
inline void Control::set_type(::google::protobuf::int32 value) {
  
  type_ = value;
  // @@protoc_insertion_point(field_set:QRcommand.msgs.Control.type)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace QRcommand

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_control_2eproto__INCLUDED
