// 把 out/*.csv 嵌进一个自包含的 HTML 报告(无需 Python / 联网 / 任何依赖)
//
// 用法: VinsReport            (读 out/*.csv，生成 out/report.html)

#include <cstdio>
#include <cctype>
#include <string>
#include <vector>

namespace {

std::string readFile(const std::string &path) {
    FILE *f = std::fopen(path.c_str(), "rb");
    if (!f) { std::fprintf(stderr, "warn: missing %s\n", path.c_str()); return {}; }
    std::string s;
    char buf[65536];
    size_t n;
    while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) s.append(buf, n);
    std::fclose(f);
    return s;
}

// 转义成 JS 模板字符串安全的形式
std::string esc(const std::string &s) {
    std::string o;
    o.reserve(s.size() + 64);
    for (char c : s) {
        if (c == '\\' || c == '`' || c == '$') { o.push_back('\\'); o.push_back(c); }
        else if (c == '\r') continue;
        else o.push_back(c);
    }
    return o;
}

} // namespace

extern const char *kReportTemplate;   // 在 report_template.h 里

int main() {
    const std::vector<std::string> scenarios{
        "circle_out", "circle_in", "helix_3d", "stop_go"
    };
    std::vector<std::string> trajectories, updates, landmarks;
    trajectories.reserve(scenarios.size());
    updates.reserve(scenarios.size());
    landmarks.reserve(scenarios.size());
    for (const auto &scenario : scenarios) {
        trajectories.emplace_back(readFile("out/traj_" + scenario + "_base.csv"));
        updates.emplace_back(readFile("out/update_" + scenario + "_base.csv"));
        landmarks.emplace_back(readFile("out/lmk_" + scenario + ".csv"));
    }
    const std::string tri = readFile("out/triangulation_circle_out_base.csv");
    const std::string sum  = readFile("out/summary.csv");

    if (trajectories.front().empty()) {
        std::fprintf(stderr,
                     "error: out/traj_circle_out_base.csv not found. Run the multi-scenario analysis first.\n");
        return 1;
    }

    FILE *f = std::fopen("out/report.html", "wb");
    if (!f) { std::fprintf(stderr, "cannot write out/report.html\n"); return 1; }

    // 模板里用 %%DATA_XXX%% 占位
    std::string html = kReportTemplate;
    auto sub = [&](const std::string &key, const std::string &val) {
        const auto p = html.find(key);
        if (p != std::string::npos) html.replace(p, key.size(), val);
    };
    for (size_t i = 0; i < scenarios.size(); ++i) {
        std::string key = scenarios[i];
        for (auto &c : key) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        sub("%%DATA_TRAJ_" + key + "%%", esc(trajectories[i]));
        sub("%%DATA_UPDATE_" + key + "%%", esc(updates[i]));
        sub("%%DATA_LMK_" + key + "%%", esc(landmarks[i]));
    }
    sub("%%DATA_TRIANGULATION%%", esc(tri));
    sub("%%DATA_SUMMARY%%", esc(sum));

    std::fwrite(html.data(), 1, html.size(), f);
    std::fclose(f);
    std::printf("wrote out/report.html (%zu KB)\n", html.size() / 1024);
    return 0;
}
