file(READ "config/99-realsense-libusb.rules" contents HEX)

set(UDEV_HEADER "${CMAKE_CURRENT_BINARY_DIR}/udev-rules.h")

# Build the generated header in memory and write it once.  The original 2.50.0
# script appended one byte per file operation, which can intermittently fail on
# Windows because virus scanners/indexers briefly lock the output file.
set(UDEV_HEADER_CONTENT
  "#ifndef __UDEV_RULES_H__\n"
  "#define __UDEV_RULES_H__\n"
  "#ifdef __cplusplus\n"
  "extern \"C\" {\n"
  "#endif\n"
  "const char realsense_udev_rules[] = {")

string(LENGTH "${contents}" contents_length)
math(EXPR contents_length "${contents_length} - 1")

foreach(iter RANGE 0 ${contents_length} 2)
  string(SUBSTRING ${contents} ${iter} 2 line)
  set(UDEV_HEADER_CONTENT "${UDEV_HEADER_CONTENT}0x${line},")
endforeach()

set(UDEV_HEADER_CONTENT
  "${UDEV_HEADER_CONTENT}};\n"
  "#ifdef __cplusplus\n"
  "}\n"
  "#endif\n"
  "#endif//\n")

file(WRITE "${UDEV_HEADER}" "${UDEV_HEADER_CONTENT}")
