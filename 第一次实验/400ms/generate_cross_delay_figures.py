from pathlib import Path
from xml.sax.saxutils import escape


OUT = Path(__file__).resolve().parent / "figures"
OUT.mkdir(exist_ok=True)


def svg_begin(width, height, title):
    return [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        '<style>text{font-family:"Microsoft YaHei","Noto Sans CJK SC",Arial,sans-serif;fill:#202124}.title{font-size:22px;font-weight:700}.axis{font-size:13px;fill:#5f6368}.label{font-size:14px}.value{font-size:13px;font-weight:600}.note{font-size:12px;fill:#5f6368}.grid{stroke:#dfe3e8;stroke-width:1}.axisline{stroke:#70757a;stroke-width:1.2}</style>',
        f'<text x="40" y="35" class="title">{escape(title)}</text>',
    ]


def write(name, parts):
    parts.append("</svg>")
    (OUT / name).write_text("\n".join(parts), encoding="utf-8")


def latency_decomposition():
    width, height = 1000, 540
    labels = ["baseline", "100 ms", "300 ms", "400 ms-1727", "400 ms-1739"]
    upstream = [266.629, 293.177, 290.246, 309.824, 836.209]
    postcontrol = [77.028, 170.416, 368.671, 499.444, 496.592]
    ymax = 1400
    left, right, top, bottom = 95, 35, 75, 95
    pw, ph = width-left-right, height-top-bottom
    p = svg_begin(width, height, "图 1  t₁ 到持续减速的时延分解")
    for y in range(0, ymax+1, 200):
        py = top + ph - y/ymax*ph
        p += [f'<line x1="{left}" y1="{py:.1f}" x2="{left+pw}" y2="{py:.1f}" class="grid"/>',
              f'<text x="{left-12}" y="{py+5:.1f}" text-anchor="end" class="axis">{y}</text>']
    p += [f'<line x1="{left}" y1="{top}" x2="{left}" y2="{top+ph}" class="axisline"/>',
          f'<line x1="{left}" y1="{top+ph}" x2="{left+pw}" y2="{top+ph}" class="axisline"/>',
          f'<text x="24" y="{top+ph/2}" transform="rotate(-90 24 {top+ph/2})" text-anchor="middle" class="axis">时延 (ms)</text>']
    step, bw = pw/len(labels), 92
    for i, label in enumerate(labels):
        cx = left + step*(i+0.5)
        h1, h2 = upstream[i]/ymax*ph, postcontrol[i]/ymax*ph
        y1 = top+ph-h1
        y2 = y1-h2
        p += [f'<rect x="{cx-bw/2:.1f}" y="{y1:.1f}" width="{bw}" height="{h1:.1f}" fill="#4e79a7"/>',
              f'<rect x="{cx-bw/2:.1f}" y="{y2:.1f}" width="{bw}" height="{h2:.1f}" fill="#f28e2b"/>',
              f'<text x="{cx:.1f}" y="{y2-8:.1f}" text-anchor="middle" class="value">{upstream[i]+postcontrol[i]:.1f}</text>',
              f'<text x="{cx:.1f}" y="{top+ph+26}" text-anchor="middle" class="label">{escape(label)}</text>']
    p += ['<rect x="650" y="50" width="15" height="15" fill="#4e79a7"/><text x="672" y="63" class="label">t₁→Control</text>',
          '<rect x="780" y="50" width="15" height="15" fill="#f28e2b"/><text x="802" y="63" class="label">Control→持续减速</text>',
          '<text x="95" y="515" class="note">注：400 ms-1739 的上游段显著增大，说明该次碰撞并非只由 400 ms 注入造成。</text>']
    write("fig1_latency_decomposition.svg", p)


def response_travel():
    width, height = 1000, 520
    labels = ["baseline", "100 ms", "300 ms", "400 ms-1727", "400 ms-1739"]
    values = [4.619, 6.287, 9.479, 11.696, 21.255]
    colors = ["#4e79a7", "#4e79a7", "#4e79a7", "#e15759", "#e15759"]
    ymax = 24
    left, right, top, bottom = 90, 35, 75, 95
    pw, ph = width-left-right, height-top-bottom
    p = svg_begin(width, height, "图 2  持续减速开始前的额外行驶距离")
    for y in range(0, ymax+1, 4):
        py = top + ph - y/ymax*ph
        p += [f'<line x1="{left}" y1="{py:.1f}" x2="{left+pw}" y2="{py:.1f}" class="grid"/>',
              f'<text x="{left-12}" y="{py+5:.1f}" text-anchor="end" class="axis">{y}</text>']
    p += [f'<line x1="{left}" y1="{top}" x2="{left}" y2="{top+ph}" class="axisline"/>',
          f'<line x1="{left}" y1="{top+ph}" x2="{left+pw}" y2="{top+ph}" class="axisline"/>',
          f'<text x="24" y="{top+ph/2}" transform="rotate(-90 24 {top+ph/2})" text-anchor="middle" class="axis">响应阶段行驶距离 (m)</text>']
    step, bw = pw/len(labels), 92
    for i, (label, val) in enumerate(zip(labels, values)):
        cx = left + step*(i+0.5)
        h = val/ymax*ph
        y = top+ph-h
        p += [f'<rect x="{cx-bw/2:.1f}" y="{y:.1f}" width="{bw}" height="{h:.1f}" rx="3" fill="{colors[i]}"/>',
              f'<text x="{cx:.1f}" y="{y-8:.1f}" text-anchor="middle" class="value">{val:.3f}</text>',
              f'<text x="{cx:.1f}" y="{top+ph+26}" text-anchor="middle" class="label">{escape(label)}</text>']
        if i >= 3:
            p.append(f'<text x="{cx:.1f}" y="{top+ph+48}" text-anchor="middle" class="value" fill="#e15759">碰撞</text>')
    p += ['<rect x="700" y="50" width="15" height="15" fill="#4e79a7"/><text x="722" y="63" class="label">未碰撞组均值</text>',
          '<rect x="835" y="50" width="15" height="15" fill="#e15759"/><text x="857" y="63" class="label">碰撞场景</text>']
    write("fig2_response_travel.svg", p)


def backlog():
    width, height = 1050, 560
    labels = ["441–460", "461–480", "481–500", "501–520", "521–540", "541–560", "561–580", "581–600", "601–620", "621–641", "642–649"]
    values = [258, 289, 254, 308, 655, 1001, 803, 1880, 1916, 1988, 2006]
    ymax = 2200
    left, right, top, bottom = 90, 35, 75, 115
    pw, ph = width-left-right, height-top-bottom
    p = svg_begin(width, height, "图 3  400 ms-1739 全生命周期 Fusion 帧时延持续堆积")
    for y in range(0, ymax+1, 400):
        py = top + ph - y/ymax*ph
        p += [f'<line x1="{left}" y1="{py:.1f}" x2="{left+pw}" y2="{py:.1f}" class="grid"/>',
              f'<text x="{left-12}" y="{py+5:.1f}" text-anchor="end" class="axis">{y}</text>']
    p += [f'<line x1="{left}" y1="{top}" x2="{left}" y2="{top+ph}" class="axisline"/>',
          f'<line x1="{left}" y1="{top+ph}" x2="{left+pw}" y2="{top+ph}" class="axisline"/>',
          f'<text x="24" y="{top+ph/2}" transform="rotate(-90 24 {top+ph/2})" text-anchor="middle" class="axis">Fusion header_time − 点云时间戳 (ms)</text>']
    step = pw/(len(labels)-1)
    pts=[]
    for i, (label, val) in enumerate(zip(labels, values)):
        x=left+step*i; y=top+ph-val/ymax*ph
        pts.append(f"{x:.1f},{y:.1f}")
        p += [f'<circle cx="{x:.1f}" cy="{y:.1f}" r="5" fill="#e15759"/>',
              f'<text x="{x:.1f}" y="{y-10:.1f}" text-anchor="middle" class="value">{val}</text>',
              f'<text x="{x:.1f}" y="{top+ph+26}" transform="rotate(35 {x:.1f} {top+ph+26})" text-anchor="start" class="axis">{label}</text>']
    p.append(f'<polyline points="{" ".join(pts)}" fill="none" stroke="#e15759" stroke-width="3"/>')
    # 500 ms reference line
    y500=top+ph-500/ymax*ph
    p += [f'<line x1="{left}" y1="{y500:.1f}" x2="{left+pw}" y2="{y500:.1f}" stroke="#f28e2b" stroke-width="2" stroke-dasharray="7,6"/>',
          f'<text x="{left+pw-4}" y="{y500-7:.1f}" text-anchor="end" class="value" fill="#f28e2b">500 ms</text>',
          '<text x="90" y="535" class="note">横轴为连续 Fusion 序列号分箱；后半段稳定升至约 1.9–2.0 s，属于持续排队而非单帧离群值。</text>']
    write("fig3_1739_fusion_backlog.svg", p)


if __name__ == "__main__":
    latency_decomposition()
    response_travel()
    backlog()
    print(f"wrote figures to {OUT}")
