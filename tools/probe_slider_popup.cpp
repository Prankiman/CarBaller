// Headless probe for the right-click "type a value" slider popup. It drives
// the real typedValuePopup()/sliderF() from settings_ui.cpp with synthetic
// ImGui events (no window, no GPU) so the popup's contract is pinned down by
// a test rather than by reading the hint text: what Enter does, what Escape
// does, what clicking away does, and what the hint promises.
//
// The context is configured the same way app.cpp configures it
// (NavEnableKeyboard) because ImGui routes Escape through the nav code when
// that flag is set - a popup can behave differently without it.
//
// Build:
//   c++ -std=c++20 -O2 -w -I. -Isrc -Ithird_party $(sdl2-config --cflags) \
//       tools/probe_slider_popup.cpp src/input/input.cpp src/settings/settings.cpp \
//       build/libimgui.a $(sdl2-config --libs) -o /tmp/opencode/probe_slider_popup \
//       && /tmp/opencode/probe_slider_popup
// exit 0 = all checks ok
#include "src/ui/settings_ui.cpp"   // typedValuePopup() + sliderF() live in an anon namespace here

#include <imgui_internal.h>         // OpenPopupStack, to observe popup open/closed

#include <cstdio>
#include <string>

namespace {
int failed = 0;
float g_v = 0.0f;                   // the value under the slider
ImVec2 g_sliderMin, g_sliderMax;    // where the slider ended up, for clicks
bool g_applied = false;             // did the popup report an apply this frame
bool g_entryOpen = false;           // the popup's own "I am up" flag

void check(bool ok, const char* what, const char* detail) {
    if (!ok) failed++;
    std::printf("%-56s %s  %s\n", what, ok ? "ok  " : "FAIL", detail);
}
// detail built from the live value, so a failure reports what really happened
std::string at(const char* what) {
    char b[96];
    std::snprintf(b, sizeof b, "%s (value=%.6g)", what, double(g_v));
    return b;
}

int popupDepth() { return GImGui->OpenPopupStack.Size; }

void pump() {  // one frame: events -> NewFrame -> our widgets -> EndFrame
    ImGui::NewFrame();
    ImGui::SetNextWindowPos(ImVec2(100, 100));
    ImGui::SetNextWindowSize(ImVec2(400, 300));
    ImGui::Begin("Settings", nullptr,
                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("FOV");
    ImGui::SameLine();
    ImGui::PushItemWidth(-FLT_MIN);
    if (ImGui::SliderFloat("##FOV", &g_v, 60.0f, 110.0f, "%.0f",
                           ImGuiSliderFlags_AlwaysClamp))
        g_applied = true;
    g_applied = typedValuePopup("##FOV", &g_v, 60.0f, 110.0f, g_entryOpen) ||
                g_applied;
    g_sliderMin = ImGui::GetItemRectMin();
    g_sliderMax = ImGui::GetItemRectMax();
    ImGui::PopItemWidth();
    ImGui::End();
    ImGui::EndFrame();
}

void mouse(float x, float y, int btn, bool down) {
    ImGui::GetIO().AddMousePosEvent(x, y);
    ImGui::GetIO().AddMouseButtonEvent(btn, down);
}

// Move to (x,y) and right-click, the way a player would.
void openPopupAt(float x, float y) {
    mouse(x, y, 1, true);
    pump();
    mouse(ImGui::GetIO().MousePos.x, ImGui::GetIO().MousePos.y, 1, false);
    pump();
}

void type(const char* s) {
    ImGui::GetIO().AddInputCharactersUTF8(s);
    pump();
}

void key(ImGuiKey k) {
    ImGui::GetIO().AddKeyEvent(k, true);
    pump();
    ImGui::GetIO().AddKeyEvent(k, false);
    pump();
}
}  // namespace

int main() {
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    // Match app.cpp: the game turns keyboard nav on, which is what routes
    // Escape through ImGui's NavUpdateCancelRequest() instead of leaving it
    // for the caller. Testing without this flag would test a config we ship.
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.DeltaTime = 1.0f / 60.0f;
    io.DisplaySize = ImVec2(800, 600);
    unsigned char* px; int fw, fh;
    io.Fonts->GetTexDataAsRGBA32(&px, &fw, &fh);   // build the atlas headless

    // ---- 1. right-click opens the popup and seeds it with the current value
    g_v = 90.0f;
    pump();                            // first frame: the slider picks a real rect
    const float cx = (g_sliderMin.x + g_sliderMax.x) * 0.5f;
    const float cy = (g_sliderMin.y + g_sliderMax.y) * 0.5f;
    std::printf("      [slider rect] (%.0f,%.0f)-(%.0f,%.0f)\n", g_sliderMin.x,
                g_sliderMin.y, g_sliderMax.x, g_sliderMax.y);
    openPopupAt(cx, cy);
    check(popupDepth() == 1, "right-click on the slider opens the popup",
          "popup stack = 1");
    std::printf("      [after open] value=%.6g\n", double(g_v));

    // ---- 2. typing a number then Enter applies and closes
    type("75");
    key(ImGuiKey_Enter);
    check(popupDepth() == 0, "Enter closes the popup", "popup stack = 0");
    check(g_v == 75.0f, "Enter applies what was typed", at("typed 75").c_str());

    // ---- 3. a value past the slider's ends snaps to the end
    openPopupAt(cx, cy);
    type("9999");
    key(ImGuiKey_Enter);
    check(g_v == 110.0f, "typing 9999 snaps to the 110 max", at("typed 9999").c_str());

    // ---- 4. text that is not a number leaves the setting alone
    openPopupAt(cx, cy);
    io.AddInputCharactersUTF8("-.e");
    pump();
    key(ImGuiKey_Enter);
    check(g_v == 110.0f, "garbage text does not change the value", at("typed '-.e'").c_str());

    // ---- 5. CLICK AWAY, inside the settings window (empty space).
    // Not a close mechanism, and above all not a way to apply: the hint names
    // Enter and Esc only. Whatever it does to the popup, it must never write.
    openPopupAt(cx, cy);
    type("80");
    g_applied = false;
    mouse(300.0f, 360.0f, 0, true);   // empty space, still inside the window
    pump();
    mouse(ImGui::GetIO().MousePos.x, ImGui::GetIO().MousePos.y, 0, false);
    pump();
    std::printf("      [click away, inside window] popup=%d value=%.0f applied=%d\n",
                popupDepth(), g_v, (int)g_applied);
    check(!g_applied, "clicking away inside the window never applies",
          g_applied ? "value was applied" : "value untouched");

    // ---- 6. CLICK AWAY, outside the settings window entirely
    type("82");
    g_applied = false;
    mouse(700.0f, 550.0f, 0, true);   // void, far from the window
    pump();
    mouse(ImGui::GetIO().MousePos.x, ImGui::GetIO().MousePos.y, 0, false);
    pump();
    std::printf("      [click away, outside window] popup=%d value=%.0f applied=%d\n",
                popupDepth(), g_v, (int)g_applied);
    check(!g_applied, "outside click does not apply either",
          g_applied ? "value was applied" : "value untouched");
    if (popupDepth() != 0) {          // leave a clean slate for Escape
        mouse(700.0f, 550.0f, 0, true);
        pump();
        mouse(ImGui::GetIO().MousePos.x, ImGui::GetIO().MousePos.y, 0, false);
        pump();
    }

    // ---- 7. Escape: closes the popup, throws the edit away
    openPopupAt(cx, cy);
    type("68");
    g_applied = false;
    key(ImGuiKey_Escape);
    std::printf("      [Escape] popup=%d value=%.0f applied=%d entryOpen=%d\n",
                popupDepth(), g_v, (int)g_applied, (int)g_entryOpen);
    check(popupDepth() == 0, "Escape closes the popup",
          popupDepth() == 0 ? "popup closed" : "popup still open");
    check(!g_applied && g_v == 110.0f, "Escape cancels (typed 68 not applied)",
          at("after Esc").c_str());
    check(!g_entryOpen,
          "Escape drops the app-gate flag, so a 2nd Esc closes the menu",
          "entryOpen still true - menu would be trapped open");

    // ---- 8. the flag is up for exactly as long as the popup is, which is
    // what lets app.cpp hold the menu's Escape for one press only
    openPopupAt(cx, cy);
    check(g_entryOpen, "popup open raises the app-gate flag", "entryOpen true");
    g_applied = false;
    key(ImGuiKey_Escape);
    check(g_entryOpen == false, "flag falls the frame the popup closes",
          "entryOpen true after close");

    // ---- 8. the hint text the player is shown must match what we just tested
    {
        // The hint lives in typedValuePopup(); read the source contract by
        // rendering one frame of the popup and capturing the line.
        std::FILE* f = std::fopen("src/ui/settings_ui.cpp", "r");
        std::string src;
        if (f) {
            char line[512];
            while (std::fgets(line, sizeof line, f)) src += line;
            std::fclose(f);
        }
        const bool hasHint = src.find("Enter = apply, Esc = cancel") != std::string::npos;
        const bool staleClickHint =
            src.find("Esc/click away = cancel") != std::string::npos ||
            src.find("clicking away from the field applies it") != std::string::npos;
        check(hasHint, "popup hint tells the player Enter = apply, Esc = cancel",
              hasHint ? "hint present" : "hint text missing");
        check(!staleClickHint, "no stale click-away promise left in the hint",
              staleClickHint ? "old click-away wording still present" : "wording clean");
    }

    ImGui::DestroyContext();
    std::printf("\n%s\n",
                failed ? "SLIDER POPUP REGRESSION" : "slider popup contract holds");
    return failed ? 1 : 0;
}
