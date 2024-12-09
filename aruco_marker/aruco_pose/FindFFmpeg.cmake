find_path(FFMPEG_INCLUDE_DIR
  NAMES libavformat/avformat.h
  PATHS /usr/include /usr/local/include
)

find_library(FFMPEG_LIBRARIES
  NAMES avformat avcodec avutil avdevice
  PATHS /usr/lib /usr/local/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FFmpeg DEFAULT_MSG FFMPEG_LIBRARIES FFMPEG_INCLUDE_DIR)

mark_as_advanced(FFMPEG_INCLUDE_DIR FFMPEG_LIBRARIES)
