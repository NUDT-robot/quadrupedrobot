// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: jointsposition.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "include/gazebo/message/jointsposition.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace QRsensor {
namespace msgs {

namespace {

const ::google::protobuf::Descriptor* LimbJoints_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  LimbJoints_reflection_ = NULL;
const ::google::protobuf::Descriptor* JointsPosition_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  JointsPosition_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_jointsposition_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AssignDesc_jointsposition_2eproto() {
  protobuf_AddDesc_jointsposition_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "jointsposition.proto");
  GOOGLE_CHECK(file != NULL);
  LimbJoints_descriptor_ = file->message_type(0);
  static const int LimbJoints_offsets_[3] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(LimbJoints, theta1_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(LimbJoints, theta2_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(LimbJoints, theta3_),
  };
  LimbJoints_reflection_ =
    ::google::protobuf::internal::GeneratedMessageReflection::NewGeneratedMessageReflection(
      LimbJoints_descriptor_,
      LimbJoints::default_instance_,
      LimbJoints_offsets_,
      -1,
      -1,
      -1,
      sizeof(LimbJoints),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(LimbJoints, _internal_metadata_),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(LimbJoints, _is_default_instance_));
  JointsPosition_descriptor_ = file->message_type(1);
  static const int JointsPosition_offsets_[1] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(JointsPosition, limbjoints_),
  };
  JointsPosition_reflection_ =
    ::google::protobuf::internal::GeneratedMessageReflection::NewGeneratedMessageReflection(
      JointsPosition_descriptor_,
      JointsPosition::default_instance_,
      JointsPosition_offsets_,
      -1,
      -1,
      -1,
      sizeof(JointsPosition),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(JointsPosition, _internal_metadata_),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(JointsPosition, _is_default_instance_));
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_jointsposition_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
      LimbJoints_descriptor_, &LimbJoints::default_instance());
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
      JointsPosition_descriptor_, &JointsPosition::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_jointsposition_2eproto() {
  delete LimbJoints::default_instance_;
  delete LimbJoints_reflection_;
  delete JointsPosition::default_instance_;
  delete JointsPosition_reflection_;
}

void protobuf_AddDesc_jointsposition_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AddDesc_jointsposition_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\024jointsposition.proto\022\rQRsensor.msgs\"<\n"
    "\nLimbJoints\022\016\n\006theta1\030\001 \001(\001\022\016\n\006theta2\030\002 "
    "\001(\001\022\016\n\006theta3\030\003 \001(\001\"\?\n\016JointsPosition\022-\n"
    "\nlimbjoints\030\001 \003(\0132\031.QRsensor.msgs.LimbJo"
    "intsb\006proto3", 172);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "jointsposition.proto", &protobuf_RegisterTypes);
  LimbJoints::default_instance_ = new LimbJoints();
  JointsPosition::default_instance_ = new JointsPosition();
  LimbJoints::default_instance_->InitAsDefaultInstance();
  JointsPosition::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_jointsposition_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_jointsposition_2eproto {
  StaticDescriptorInitializer_jointsposition_2eproto() {
    protobuf_AddDesc_jointsposition_2eproto();
  }
} static_descriptor_initializer_jointsposition_2eproto_;

// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int LimbJoints::kTheta1FieldNumber;
const int LimbJoints::kTheta2FieldNumber;
const int LimbJoints::kTheta3FieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

LimbJoints::LimbJoints()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:QRsensor.msgs.LimbJoints)
}

void LimbJoints::InitAsDefaultInstance() {
  _is_default_instance_ = true;
}

LimbJoints::LimbJoints(const LimbJoints& from)
  : ::google::protobuf::Message(),
    _internal_metadata_(NULL) {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:QRsensor.msgs.LimbJoints)
}

void LimbJoints::SharedCtor() {
    _is_default_instance_ = false;
  _cached_size_ = 0;
  theta1_ = 0;
  theta2_ = 0;
  theta3_ = 0;
}

LimbJoints::~LimbJoints() {
  // @@protoc_insertion_point(destructor:QRsensor.msgs.LimbJoints)
  SharedDtor();
}

void LimbJoints::SharedDtor() {
  if (this != default_instance_) {
  }
}

void LimbJoints::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* LimbJoints::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return LimbJoints_descriptor_;
}

const LimbJoints& LimbJoints::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_jointsposition_2eproto();
  return *default_instance_;
}

LimbJoints* LimbJoints::default_instance_ = NULL;

LimbJoints* LimbJoints::New(::google::protobuf::Arena* arena) const {
  LimbJoints* n = new LimbJoints;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void LimbJoints::Clear() {
// @@protoc_insertion_point(message_clear_start:QRsensor.msgs.LimbJoints)
#if defined(__clang__)
#define ZR_HELPER_(f) \
  _Pragma("clang diagnostic push") \
  _Pragma("clang diagnostic ignored \"-Winvalid-offsetof\"") \
  __builtin_offsetof(LimbJoints, f) \
  _Pragma("clang diagnostic pop")
#else
#define ZR_HELPER_(f) reinterpret_cast<char*>(\
  &reinterpret_cast<LimbJoints*>(16)->f)
#endif

#define ZR_(first, last) do {\
  ::memset(&first, 0,\
           ZR_HELPER_(last) - ZR_HELPER_(first) + sizeof(last));\
} while (0)

  ZR_(theta1_, theta3_);

#undef ZR_HELPER_
#undef ZR_

}

bool LimbJoints::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:QRsensor.msgs.LimbJoints)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional double theta1 = 1;
      case 1: {
        if (tag == 9) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &theta1_)));

        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(17)) goto parse_theta2;
        break;
      }

      // optional double theta2 = 2;
      case 2: {
        if (tag == 17) {
         parse_theta2:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &theta2_)));

        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(25)) goto parse_theta3;
        break;
      }

      // optional double theta3 = 3;
      case 3: {
        if (tag == 25) {
         parse_theta3:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &theta3_)));

        } else {
          goto handle_unusual;
        }
        if (input->ExpectAtEnd()) goto success;
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormatLite::SkipField(input, tag));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:QRsensor.msgs.LimbJoints)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:QRsensor.msgs.LimbJoints)
  return false;
#undef DO_
}

void LimbJoints::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:QRsensor.msgs.LimbJoints)
  // optional double theta1 = 1;
  if (this->theta1() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->theta1(), output);
  }

  // optional double theta2 = 2;
  if (this->theta2() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->theta2(), output);
  }

  // optional double theta3 = 3;
  if (this->theta3() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(3, this->theta3(), output);
  }

  // @@protoc_insertion_point(serialize_end:QRsensor.msgs.LimbJoints)
}

::google::protobuf::uint8* LimbJoints::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:QRsensor.msgs.LimbJoints)
  // optional double theta1 = 1;
  if (this->theta1() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->theta1(), target);
  }

  // optional double theta2 = 2;
  if (this->theta2() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->theta2(), target);
  }

  // optional double theta3 = 3;
  if (this->theta3() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(3, this->theta3(), target);
  }

  // @@protoc_insertion_point(serialize_to_array_end:QRsensor.msgs.LimbJoints)
  return target;
}

int LimbJoints::ByteSize() const {
// @@protoc_insertion_point(message_byte_size_start:QRsensor.msgs.LimbJoints)
  int total_size = 0;

  // optional double theta1 = 1;
  if (this->theta1() != 0) {
    total_size += 1 + 8;
  }

  // optional double theta2 = 2;
  if (this->theta2() != 0) {
    total_size += 1 + 8;
  }

  // optional double theta3 = 3;
  if (this->theta3() != 0) {
    total_size += 1 + 8;
  }

  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void LimbJoints::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:QRsensor.msgs.LimbJoints)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  const LimbJoints* source = 
      ::google::protobuf::internal::DynamicCastToGenerated<const LimbJoints>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:QRsensor.msgs.LimbJoints)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:QRsensor.msgs.LimbJoints)
    MergeFrom(*source);
  }
}

void LimbJoints::MergeFrom(const LimbJoints& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:QRsensor.msgs.LimbJoints)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  if (from.theta1() != 0) {
    set_theta1(from.theta1());
  }
  if (from.theta2() != 0) {
    set_theta2(from.theta2());
  }
  if (from.theta3() != 0) {
    set_theta3(from.theta3());
  }
}

void LimbJoints::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:QRsensor.msgs.LimbJoints)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void LimbJoints::CopyFrom(const LimbJoints& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:QRsensor.msgs.LimbJoints)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool LimbJoints::IsInitialized() const {

  return true;
}

void LimbJoints::Swap(LimbJoints* other) {
  if (other == this) return;
  InternalSwap(other);
}
void LimbJoints::InternalSwap(LimbJoints* other) {
  std::swap(theta1_, other->theta1_);
  std::swap(theta2_, other->theta2_);
  std::swap(theta3_, other->theta3_);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  std::swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata LimbJoints::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = LimbJoints_descriptor_;
  metadata.reflection = LimbJoints_reflection_;
  return metadata;
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// LimbJoints

// optional double theta1 = 1;
void LimbJoints::clear_theta1() {
  theta1_ = 0;
}
 double LimbJoints::theta1() const {
  // @@protoc_insertion_point(field_get:QRsensor.msgs.LimbJoints.theta1)
  return theta1_;
}
 void LimbJoints::set_theta1(double value) {
  
  theta1_ = value;
  // @@protoc_insertion_point(field_set:QRsensor.msgs.LimbJoints.theta1)
}

// optional double theta2 = 2;
void LimbJoints::clear_theta2() {
  theta2_ = 0;
}
 double LimbJoints::theta2() const {
  // @@protoc_insertion_point(field_get:QRsensor.msgs.LimbJoints.theta2)
  return theta2_;
}
 void LimbJoints::set_theta2(double value) {
  
  theta2_ = value;
  // @@protoc_insertion_point(field_set:QRsensor.msgs.LimbJoints.theta2)
}

// optional double theta3 = 3;
void LimbJoints::clear_theta3() {
  theta3_ = 0;
}
 double LimbJoints::theta3() const {
  // @@protoc_insertion_point(field_get:QRsensor.msgs.LimbJoints.theta3)
  return theta3_;
}
 void LimbJoints::set_theta3(double value) {
  
  theta3_ = value;
  // @@protoc_insertion_point(field_set:QRsensor.msgs.LimbJoints.theta3)
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int JointsPosition::kLimbjointsFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

JointsPosition::JointsPosition()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:QRsensor.msgs.JointsPosition)
}

void JointsPosition::InitAsDefaultInstance() {
  _is_default_instance_ = true;
}

JointsPosition::JointsPosition(const JointsPosition& from)
  : ::google::protobuf::Message(),
    _internal_metadata_(NULL) {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:QRsensor.msgs.JointsPosition)
}

void JointsPosition::SharedCtor() {
    _is_default_instance_ = false;
  _cached_size_ = 0;
}

JointsPosition::~JointsPosition() {
  // @@protoc_insertion_point(destructor:QRsensor.msgs.JointsPosition)
  SharedDtor();
}

void JointsPosition::SharedDtor() {
  if (this != default_instance_) {
  }
}

void JointsPosition::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* JointsPosition::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return JointsPosition_descriptor_;
}

const JointsPosition& JointsPosition::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_jointsposition_2eproto();
  return *default_instance_;
}

JointsPosition* JointsPosition::default_instance_ = NULL;

JointsPosition* JointsPosition::New(::google::protobuf::Arena* arena) const {
  JointsPosition* n = new JointsPosition;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void JointsPosition::Clear() {
// @@protoc_insertion_point(message_clear_start:QRsensor.msgs.JointsPosition)
  limbjoints_.Clear();
}

bool JointsPosition::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:QRsensor.msgs.JointsPosition)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // repeated .QRsensor.msgs.LimbJoints limbjoints = 1;
      case 1: {
        if (tag == 10) {
          DO_(input->IncrementRecursionDepth());
         parse_loop_limbjoints:
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtualNoRecursionDepth(
                input, add_limbjoints()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(10)) goto parse_loop_limbjoints;
        input->UnsafeDecrementRecursionDepth();
        if (input->ExpectAtEnd()) goto success;
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormatLite::SkipField(input, tag));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:QRsensor.msgs.JointsPosition)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:QRsensor.msgs.JointsPosition)
  return false;
#undef DO_
}

void JointsPosition::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:QRsensor.msgs.JointsPosition)
  // repeated .QRsensor.msgs.LimbJoints limbjoints = 1;
  for (unsigned int i = 0, n = this->limbjoints_size(); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, this->limbjoints(i), output);
  }

  // @@protoc_insertion_point(serialize_end:QRsensor.msgs.JointsPosition)
}

::google::protobuf::uint8* JointsPosition::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:QRsensor.msgs.JointsPosition)
  // repeated .QRsensor.msgs.LimbJoints limbjoints = 1;
  for (unsigned int i = 0, n = this->limbjoints_size(); i < n; i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageNoVirtualToArray(
        1, this->limbjoints(i), false, target);
  }

  // @@protoc_insertion_point(serialize_to_array_end:QRsensor.msgs.JointsPosition)
  return target;
}

int JointsPosition::ByteSize() const {
// @@protoc_insertion_point(message_byte_size_start:QRsensor.msgs.JointsPosition)
  int total_size = 0;

  // repeated .QRsensor.msgs.LimbJoints limbjoints = 1;
  total_size += 1 * this->limbjoints_size();
  for (int i = 0; i < this->limbjoints_size(); i++) {
    total_size +=
      ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
        this->limbjoints(i));
  }

  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void JointsPosition::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:QRsensor.msgs.JointsPosition)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  const JointsPosition* source = 
      ::google::protobuf::internal::DynamicCastToGenerated<const JointsPosition>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:QRsensor.msgs.JointsPosition)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:QRsensor.msgs.JointsPosition)
    MergeFrom(*source);
  }
}

void JointsPosition::MergeFrom(const JointsPosition& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:QRsensor.msgs.JointsPosition)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  limbjoints_.MergeFrom(from.limbjoints_);
}

void JointsPosition::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:QRsensor.msgs.JointsPosition)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void JointsPosition::CopyFrom(const JointsPosition& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:QRsensor.msgs.JointsPosition)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool JointsPosition::IsInitialized() const {

  return true;
}

void JointsPosition::Swap(JointsPosition* other) {
  if (other == this) return;
  InternalSwap(other);
}
void JointsPosition::InternalSwap(JointsPosition* other) {
  limbjoints_.UnsafeArenaSwap(&other->limbjoints_);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  std::swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata JointsPosition::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = JointsPosition_descriptor_;
  metadata.reflection = JointsPosition_reflection_;
  return metadata;
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// JointsPosition

// repeated .QRsensor.msgs.LimbJoints limbjoints = 1;
int JointsPosition::limbjoints_size() const {
  return limbjoints_.size();
}
void JointsPosition::clear_limbjoints() {
  limbjoints_.Clear();
}
const ::QRsensor::msgs::LimbJoints& JointsPosition::limbjoints(int index) const {
  // @@protoc_insertion_point(field_get:QRsensor.msgs.JointsPosition.limbjoints)
  return limbjoints_.Get(index);
}
::QRsensor::msgs::LimbJoints* JointsPosition::mutable_limbjoints(int index) {
  // @@protoc_insertion_point(field_mutable:QRsensor.msgs.JointsPosition.limbjoints)
  return limbjoints_.Mutable(index);
}
::QRsensor::msgs::LimbJoints* JointsPosition::add_limbjoints() {
  // @@protoc_insertion_point(field_add:QRsensor.msgs.JointsPosition.limbjoints)
  return limbjoints_.Add();
}
::google::protobuf::RepeatedPtrField< ::QRsensor::msgs::LimbJoints >*
JointsPosition::mutable_limbjoints() {
  // @@protoc_insertion_point(field_mutable_list:QRsensor.msgs.JointsPosition.limbjoints)
  return &limbjoints_;
}
const ::google::protobuf::RepeatedPtrField< ::QRsensor::msgs::LimbJoints >&
JointsPosition::limbjoints() const {
  // @@protoc_insertion_point(field_list:QRsensor.msgs.JointsPosition.limbjoints)
  return limbjoints_;
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace QRsensor

// @@protoc_insertion_point(global_scope)
