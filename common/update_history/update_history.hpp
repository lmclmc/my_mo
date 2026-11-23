#pragma once

#include <pj_git_history.hpp>

#include <cstdint>
#include <string>
#include <string_view>

namespace PjUpdateHistory {

inline std::string xmlText(std::string_view text) {
  std::string result;
  result.reserve(text.size());
  for (const char ch : text) {
    switch (ch) {
      case '&': result += "&amp;"; break;
      case '<': result += "&lt;"; break;
      case '>': result += "&gt;"; break;
      case '"': result += "&quot;"; break;
      case '\'': result += "&apos;"; break;
      default: result += ch; break;
    }
  }
  return result;
}

// This child dialog has no plugin-side events or inputs to apply. Its Close
// button is bound by the host to the child dialog, leaving the caller open.
inline std::string dialogUi() {
  static std::uint64_t request_serial = 0;
  const auto serial = ++request_serial;
  std::string ui = R"PJ_HISTORY(<?xml version="1.0" encoding="UTF-8"?>
<ui version="4.0">
 <class>MotionStudioUpdateHistory</class>
 <widget class="QWidget" name="MotionStudioUpdateHistory">
  <property name="geometry"><rect><x>0</x><y>0</y><width>880</width><height>620</height></rect></property>
  <property name="minimumSize"><size><width>620</width><height>400</height></size></property>
  <property name="windowTitle"><string>MotionStudio 更新记录</string></property>
  <layout class="QVBoxLayout" name="historyLayout">
   <property name="spacing"><number>10</number></property>
   <item>
    <widget class="QLabel" name="historyDescriptionLabel">
     <property name="text"><string>当前构建版本的提交记录；在线最新记录请打开 GitLab。</string></property>
     <property name="wordWrap"><bool>true</bool></property>
    </widget>
   </item>
   <item>
    <widget class="QLabel" name="historyOnlineLabel">
     <property name="text"><string>&lt;a style="color: #6aa2c4;" href="https://gitlab.magiclab.top/a2506magicbotgen2/motionstudio_v2/-/commits/develop"&gt;在 GitLab 查看完整更新记录（develop）↗&lt;/a&gt;</string></property>
     <property name="openExternalLinks"><bool>true</bool></property>
     <property name="textInteractionFlags"><set>Qt::TextBrowserInteraction</set></property>
    </widget>
   </item>
   <item>
    <widget class="QTextBrowser" name="historyTextBrowser">
     <property name="readOnly"><bool>true</bool></property>
     <property name="openExternalLinks"><bool>true</bool></property>
     <property name="openLinks"><bool>true</bool></property>
     <property name="html"><string>)PJ_HISTORY" + xmlText(kGitHistoryHtml) + R"PJ_HISTORY(</string></property>
    </widget>
   </item>
   <item>
    <widget class="QDialogButtonBox" name="buttonBox">
     <property name="standardButtons"><set>QDialogButtonBox::Close</set></property>
    </widget>
   </item>
  </layout>
 </widget>
 <resources/>
 <connections/>
</ui>)PJ_HISTORY";
  // DialogEngine de-duplicates identical one-shot requests. A fresh inert XML
  // comment lets the same button open the history window again after it closes.
  const auto closing_tag = ui.rfind("</ui>");
  ui.insert(closing_tag, "<!-- update-history-request-" + std::to_string(serial) + " -->");
  return ui;
}

}  // namespace PjUpdateHistory
