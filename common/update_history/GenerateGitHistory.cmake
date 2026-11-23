# Called by EmbedGitHistory.cmake. Git is optional for source archives/build hosts.
if(NOT DEFINED PJ_HISTORY_OUTPUT OR PJ_HISTORY_OUTPUT STREQUAL "")
  message(FATAL_ERROR "PJ_HISTORY_OUTPUT is required")
endif()

function(_pj_write_history_header CONTENT)
  if(EXISTS "${PJ_HISTORY_OUTPUT}")
    file(READ "${PJ_HISTORY_OUTPUT}" _previous)
    if(_previous STREQUAL CONTENT)
      return()
    endif()
  endif()
  get_filename_component(_output_dir "${PJ_HISTORY_OUTPUT}" DIRECTORY)
  file(MAKE_DIRECTORY "${_output_dir}")
  file(WRITE "${PJ_HISTORY_OUTPUT}" "${CONTENT}")
endfunction()

# pack.sh captures one snapshot for all three builds. Standalone builds read Git below.
if(DEFINED ENV{PJ_PACK_GIT_HISTORY})
  file(READ "$ENV{PJ_PACK_GIT_HISTORY}" _snapshot)
  _pj_write_history_header("${_snapshot}")
  return()
endif()

function(_pj_history_escape VALUE OUTPUT)
  string(REPLACE "&" "&amp;" _escaped "${VALUE}")
  string(REPLACE "<" "&lt;" _escaped "${_escaped}")
  string(REPLACE ">" "&gt;" _escaped "${_escaped}")
  string(REPLACE "\"" "&quot;" _escaped "${_escaped}")
  string(REPLACE "'" "&#39;" _escaped "${_escaped}")
  # XML 1.0 forbids these controls; Git permits them in commit messages.
  foreach(_code RANGE 1 31)
    if(NOT _code EQUAL 9 AND NOT _code EQUAL 10 AND NOT _code EQUAL 13)
      string(ASCII ${_code} _control)
      string(REPLACE "${_control}" "" _escaped "${_escaped}")
    endif()
  endforeach()
  set(${OUTPUT} "${_escaped}" PARENT_SCOPE)
endfunction()

function(_pj_history_pop_line INPUT FIRST REST)
  string(FIND "${INPUT}" "\n" _newline)
  if(_newline LESS 0)
    set(${FIRST} "${INPUT}" PARENT_SCOPE)
    set(${REST} "" PARENT_SCOPE)
  else()
    string(SUBSTRING "${INPUT}" 0 ${_newline} _first)
    math(EXPR _next "${_newline} + 1")
    string(SUBSTRING "${INPUT}" ${_next} -1 _rest)
    set(${FIRST} "${_first}" PARENT_SCOPE)
    set(${REST} "${_rest}" PARENT_SCOPE)
  endif()
endfunction()

set(_gitlab "https://gitlab.magiclab.top/a2506magicbotgen2/motionstudio_v2")
set(_html "")
set(_commits "")
set(_git_result 1)
if(PJ_HISTORY_GIT AND EXISTS "${PJ_HISTORY_GIT}")
  # Container builds may run as root against a user-owned bind mount. Trust
  # only this explicitly selected source directory for each command, without
  # modifying global Git configuration. Resolve symlinks to match Git's path.
  file(REAL_PATH "${PJ_HISTORY_SOURCE_DIR}" _repository_path)
  execute_process(
    COMMAND "${PJ_HISTORY_GIT}" --no-pager -c "safe.directory=${_repository_path}"
      -C "${_repository_path}"
      log -n 30 --format=%H
    OUTPUT_VARIABLE _commits OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET RESULT_VARIABLE _git_result TIMEOUT 15)
endif()
if(_git_result EQUAL 0 AND NOT _commits STREQUAL "")
  execute_process(
    COMMAND "${PJ_HISTORY_GIT}" --no-pager -c "safe.directory=${_repository_path}"
      -C "${_repository_path}"
      rev-parse --abbrev-ref HEAD
    OUTPUT_VARIABLE _branch OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
  _pj_history_escape("${_branch}" _branch_html)
  string(APPEND _html "<p>分支：${_branch_html} · 最近提交（最多 30 条）</p>")
  string(REPLACE "\n" ";" _hashes "${_commits}")
  foreach(_hash IN LISTS _hashes)
    if(NOT _hash MATCHES "^[0-9a-fA-F]+$")
      continue()
    endif()
    # Author identities and the formatted subject occupy one line each. The
    # remainder is the complete body; it is never split on punctuation or on a
    # delimiter that might also occur in a Chinese/Markdown commit message.
    execute_process(
      COMMAND "${PJ_HISTORY_GIT}" --no-pager -c "safe.directory=${_repository_path}"
        -c i18n.logOutputEncoding=UTF-8
        -C "${_repository_path}" show --no-patch --no-show-signature
        --format=%aI%n%an%n%s%n%b "${_hash}" --
      OUTPUT_VARIABLE _record ERROR_QUIET RESULT_VARIABLE _show_result TIMEOUT 15)
    if(NOT _show_result EQUAL 0)
      if(PJ_HISTORY_REQUIRE_GIT)
        message(FATAL_ERROR "Could not read Git commit ${_hash}")
      endif()
      continue()
    endif()
    _pj_history_pop_line("${_record}" _date _remaining)
    _pj_history_pop_line("${_remaining}" _author _remaining)
    _pj_history_pop_line("${_remaining}" _subject _body)
    _pj_history_escape("${_date}" _date_html)
    _pj_history_escape("${_author}" _author_html)
    _pj_history_escape("${_subject}" _subject_html)
    _pj_history_escape("${_body}" _body_html)
    string(APPEND _html
      "<h3>${_subject_html}</h3>"
      "<p>${_date_html} · ${_author_html}<br/>"
      "<a style=\"color: #6aa2c4;\" href=\"${_gitlab}/-/commit/${_hash}\">${_hash}</a></p>"
      "<p style=\"white-space: pre-wrap;\">${_body_html}</p><hr/>")
  endforeach()
endif()
if(_html STREQUAL "" OR NOT _html MATCHES "<h3>")
  if(PJ_HISTORY_REQUIRE_GIT)
    message(FATAL_ERROR "Could not read Git history from ${PJ_HISTORY_SOURCE_DIR}")
  endif()
  set(_html "<p>无可用提交记录。请打开 GitLab 查看在线更新记录。</p>")
endif()

# Encode bytes rather than a raw string literal: commit contents cannot escape
# into C++, and even long histories stay below MSVC's string-literal limits.
string(HEX "${_html}" _hex)
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "static_cast<char>(0x\\1)," _bytes "${_hex}")
set(_header "#pragma once\n// Generated at build time. Do not edit.\nnamespace PjUpdateHistory {\ninline constexpr char kGitHistoryHtml[] = {\n${_bytes}0x00};\n}\n")
_pj_write_history_header("${_header}")
