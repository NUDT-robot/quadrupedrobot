// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: matlabidentification.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "include/matlab/message/matlabidentification.pb.h"

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

namespace matlab {
namespace msgs {

namespace {

const ::google::protobuf::Descriptor* Identification_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  Identification_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_matlabidentification_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AssignDesc_matlabidentification_2eproto() {
  protobuf_AddDesc_matlabidentification_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "matlabidentification.proto");
  GOOGLE_CHECK(file != NULL);
  Identification_descriptor_ = file->message_type(0);
  static const int Identification_offsets_[3] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Identification, matelement_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Identification, torque_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Identification, contactforce_),
  };
  Identification_reflection_ =
    ::google::protobuf::internal::GeneratedMessageReflection::NewGeneratedMessageReflection(
      Identification_descriptor_,
      Identification::default_instance_,
      Identification_offsets_,
      -1,
      -1,
      -1,
      sizeof(Identification),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Identification, _internal_metadata_),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Identification, _is_default_instance_));
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_matlabidentification_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
      Identification_descriptor_, &Identification::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_matlabidentification_2eproto() {
  delete Identification::default_instance_;
  delete Identification_reflection_;
}

void protobuf_AddDesc_matlabidentification_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AddDesc_matlabidentification_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\032matlabidentification.proto\022\013matlab.msg"
    "s\"J\n\016Identification\022\022\n\nMatElement\030\001 \003(\001\022"
    "\016\n\006torque\030\002 \003(\001\022\024\n\014contactforce\030\003 \003(\001b\006p"
    "roto3", 125);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "matlabidentification.proto", &protobuf_RegisterTypes);
  Identification::default_instance_ = new Identification();
  Identification::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_matlabidentification_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_matlabidentification_2eproto {
  StaticDescriptorInitializer_matlabidentification_2eproto() {
    protobuf_AddDesc_matlabidentification_2eproto();
  }
} static_descriptor_initializer_matlabidentification_2eproto_;

// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int Identification::kMatElementFieldNumber;
const int Identification::kTorqueFieldNumber;
const int Identification::kContactforceFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Identification::Identification()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:matlab.msgs.Identification)
}

void Identification::InitAsDefaultInstance() {
  _is_default_instance_ = true;
}

Identification::Identification(const Identification& from)
  : ::google::protobuf::Message(),
    _internal_metadata_(NULL) {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:matlab.msgs.Identification)
}

void Identification::SharedCtor() {
    _is_default_instance_ = false;
  _cached_size_ = 0;
}

Identification::~Identification() {
  // @@protoc_insertion_point(destructor:matlab.msgs.Identification)
  SharedDtor();
}

void Identification::SharedDtor() {
  if (this != default_instance_) {
  }
}

void Identification::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* Identification::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return Identification_descriptor_;
}

const Identification& Identification::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_matlabidentification_2eproto();
  return *default_instance_;
}

Identification* Identification::default_instance_ = NULL;

Identification* Identification::New(::google::protobuf::Arena* arena) const {
  Identification* n = new Identification;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void Identification::Clear() {
// @@protoc_insertion_point(message_clear_start:matlab.msgs.Identification)
  matelement_.Clear();
  torque_.Clear();
  contactforce_.Clear();
}

bool Identification::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:matlab.msgs.Identification)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // repeated double MatElement = 1;
      case 1: {
        if (tag == 10) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, this->mutable_matelement())));
        } else if (tag == 9) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitiveNoInline<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 1, 10, input, this->mutable_matelement())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(18)) goto parse_torque;
        break;
      }

      // repeated double torque = 2;
      case 2: {
        if (tag == 18) {
         parse_torque:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, this->mutable_torque())));
        } else if (tag == 17) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitiveNoInline<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 1, 18, input, this->mutable_torque())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(26)) goto parse_contactforce;
        break;
      }

      // repeated double contactforce = 3;
      case 3: {
        if (tag == 26) {
         parse_contactforce:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, this->mutable_contactforce())));
        } else if (tag == 25) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitiveNoInline<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 1, 26, input, this->mutable_contactforce())));
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
  // @@protoc_insertion_point(parse_success:matlab.msgs.Identification)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:matlab.msgs.Identification)
  return false;
#undef DO_
}

void Identification::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:matlab.msgs.Identification)
  // repeated double MatElement = 1;
  if (this->matelement_size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteTag(1, ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED, output);
    output->WriteVarint32(_matelement_cached_byte_size_);
  }
  for (int i = 0; i < this->matelement_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteDoubleNoTag(
      this->matelement(i), output);
  }

  // repeated double torque = 2;
  if (this->torque_size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteTag(2, ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED, output);
    output->WriteVarint32(_torque_cached_byte_size_);
  }
  for (int i = 0; i < this->torque_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteDoubleNoTag(
      this->torque(i), output);
  }

  // repeated double contactforce = 3;
  if (this->contactforce_size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteTag(3, ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED, output);
    output->WriteVarint32(_contactforce_cached_byte_size_);
  }
  for (int i = 0; i < this->contactforce_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteDoubleNoTag(
      this->contactforce(i), output);
  }

  // @@protoc_insertion_point(serialize_end:matlab.msgs.Identification)
}

::google::protobuf::uint8* Identification::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:matlab.msgs.Identification)
  // repeated double MatElement = 1;
  if (this->matelement_size() > 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteTagToArray(
      1,
      ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED,
      target);
    target = ::google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(
      _matelement_cached_byte_size_, target);
  }
  for (int i = 0; i < this->matelement_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteDoubleNoTagToArray(this->matelement(i), target);
  }

  // repeated double torque = 2;
  if (this->torque_size() > 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteTagToArray(
      2,
      ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED,
      target);
    target = ::google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(
      _torque_cached_byte_size_, target);
  }
  for (int i = 0; i < this->torque_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteDoubleNoTagToArray(this->torque(i), target);
  }

  // repeated double contactforce = 3;
  if (this->contactforce_size() > 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteTagToArray(
      3,
      ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED,
      target);
    target = ::google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(
      _contactforce_cached_byte_size_, target);
  }
  for (int i = 0; i < this->contactforce_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteDoubleNoTagToArray(this->contactforce(i), target);
  }

  // @@protoc_insertion_point(serialize_to_array_end:matlab.msgs.Identification)
  return target;
}

int Identification::ByteSize() const {
// @@protoc_insertion_point(message_byte_size_start:matlab.msgs.Identification)
  int total_size = 0;

  // repeated double MatElement = 1;
  {
    int data_size = 0;
    data_size = 8 * this->matelement_size();
    if (data_size > 0) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(data_size);
    }
    GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
    _matelement_cached_byte_size_ = data_size;
    GOOGLE_SAFE_CONCURRENT_WRITES_END();
    total_size += data_size;
  }

  // repeated double torque = 2;
  {
    int data_size = 0;
    data_size = 8 * this->torque_size();
    if (data_size > 0) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(data_size);
    }
    GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
    _torque_cached_byte_size_ = data_size;
    GOOGLE_SAFE_CONCURRENT_WRITES_END();
    total_size += data_size;
  }

  // repeated double contactforce = 3;
  {
    int data_size = 0;
    data_size = 8 * this->contactforce_size();
    if (data_size > 0) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(data_size);
    }
    GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
    _contactforce_cached_byte_size_ = data_size;
    GOOGLE_SAFE_CONCURRENT_WRITES_END();
    total_size += data_size;
  }

  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void Identification::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:matlab.msgs.Identification)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  const Identification* source = 
      ::google::protobuf::internal::DynamicCastToGenerated<const Identification>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:matlab.msgs.Identification)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:matlab.msgs.Identification)
    MergeFrom(*source);
  }
}

void Identification::MergeFrom(const Identification& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:matlab.msgs.Identification)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  matelement_.MergeFrom(from.matelement_);
  torque_.MergeFrom(from.torque_);
  contactforce_.MergeFrom(from.contactforce_);
}

void Identification::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:matlab.msgs.Identification)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Identification::CopyFrom(const Identification& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:matlab.msgs.Identification)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Identification::IsInitialized() const {

  return true;
}

void Identification::Swap(Identification* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Identification::InternalSwap(Identification* other) {
  matelement_.UnsafeArenaSwap(&other->matelement_);
  torque_.UnsafeArenaSwap(&other->torque_);
  contactforce_.UnsafeArenaSwap(&other->contactforce_);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  std::swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata Identification::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = Identification_descriptor_;
  metadata.reflection = Identification_reflection_;
  return metadata;
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// Identification

// repeated double MatElement = 1;
int Identification::matelement_size() const {
  return matelement_.size();
}
void Identification::clear_matelement() {
  matelement_.Clear();
}
 double Identification::matelement(int index) const {
  // @@protoc_insertion_point(field_get:matlab.msgs.Identification.MatElement)
  return matelement_.Get(index);
}
 void Identification::set_matelement(int index, double value) {
  matelement_.Set(index, value);
  // @@protoc_insertion_point(field_set:matlab.msgs.Identification.MatElement)
}
 void Identification::add_matelement(double value) {
  matelement_.Add(value);
  // @@protoc_insertion_point(field_add:matlab.msgs.Identification.MatElement)
}
 const ::google::protobuf::RepeatedField< double >&
Identification::matelement() const {
  // @@protoc_insertion_point(field_list:matlab.msgs.Identification.MatElement)
  return matelement_;
}
 ::google::protobuf::RepeatedField< double >*
Identification::mutable_matelement() {
  // @@protoc_insertion_point(field_mutable_list:matlab.msgs.Identification.MatElement)
  return &matelement_;
}

// repeated double torque = 2;
int Identification::torque_size() const {
  return torque_.size();
}
void Identification::clear_torque() {
  torque_.Clear();
}
 double Identification::torque(int index) const {
  // @@protoc_insertion_point(field_get:matlab.msgs.Identification.torque)
  return torque_.Get(index);
}
 void Identification::set_torque(int index, double value) {
  torque_.Set(index, value);
  // @@protoc_insertion_point(field_set:matlab.msgs.Identification.torque)
}
 void Identification::add_torque(double value) {
  torque_.Add(value);
  // @@protoc_insertion_point(field_add:matlab.msgs.Identification.torque)
}
 const ::google::protobuf::RepeatedField< double >&
Identification::torque() const {
  // @@protoc_insertion_point(field_list:matlab.msgs.Identification.torque)
  return torque_;
}
 ::google::protobuf::RepeatedField< double >*
Identification::mutable_torque() {
  // @@protoc_insertion_point(field_mutable_list:matlab.msgs.Identification.torque)
  return &torque_;
}

// repeated double contactforce = 3;
int Identification::contactforce_size() const {
  return contactforce_.size();
}
void Identification::clear_contactforce() {
  contactforce_.Clear();
}
 double Identification::contactforce(int index) const {
  // @@protoc_insertion_point(field_get:matlab.msgs.Identification.contactforce)
  return contactforce_.Get(index);
}
 void Identification::set_contactforce(int index, double value) {
  contactforce_.Set(index, value);
  // @@protoc_insertion_point(field_set:matlab.msgs.Identification.contactforce)
}
 void Identification::add_contactforce(double value) {
  contactforce_.Add(value);
  // @@protoc_insertion_point(field_add:matlab.msgs.Identification.contactforce)
}
 const ::google::protobuf::RepeatedField< double >&
Identification::contactforce() const {
  // @@protoc_insertion_point(field_list:matlab.msgs.Identification.contactforce)
  return contactforce_;
}
 ::google::protobuf::RepeatedField< double >*
Identification::mutable_contactforce() {
  // @@protoc_insertion_point(field_mutable_list:matlab.msgs.Identification.contactforce)
  return &contactforce_;
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace matlab

// @@protoc_insertion_point(global_scope)
