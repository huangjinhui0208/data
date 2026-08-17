#!/usr/bin/env python3
"""Render the TCPS-PA v4.1 Markdown source directly to a Chinese-safe PDF.

LibreOffice's isolated headless runtime does not load the host CJK fonts
reliably.  This renderer embeds a Unicode font and keeps the Markdown as the
single semantic source of truth.
"""

from __future__ import annotations

import re
from html import escape
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT, TA_RIGHT
from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import inch
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    BaseDocTemplate,
    Frame,
    HRFlowable,
    KeepTogether,
    PageBreak,
    PageTemplate,
    Paragraph,
    Preformatted,
    Spacer,
    Table,
    TableStyle,
)


ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / "自动驾驶单次运行实时性—物理安全双向诊断框架.md"
OUTPUT = ROOT / "自动驾驶单次运行通用实时性—物理安全双向诊断框架.pdf"
FONT_PATH = Path("/System/Library/Fonts/Supplemental/Arial Unicode.ttf")
FONT = "TCPSArialUnicode"

BLUE = colors.HexColor("#2E74B5")
DARK_BLUE = colors.HexColor("#1F4D78")
INK = colors.HexColor("#0B2545")
MUTED = colors.HexColor("#667085")
TABLE_FILL = colors.HexColor("#E8EEF5")
LIGHT_FILL = colors.HexColor("#F4F6F9")
RULE = colors.HexColor("#C9D5E2")
RISK = colors.HexColor("#9B1C1C")


LATEX_MAP = {
    r"\tau": "tau", r"\rho": "rho", r"\Phi": "Phi", r"\Theta": "Theta",
    r"\Delta": "Delta", r"\mathbf": "", r"\boldsymbol": "",
    r"\rightarrow": "->", r"\leftarrow": "<-", r"\mid": "|",
    r"\parallel": "||", r"\ge": ">=", r"\le": "<=", r"\in": " in ",
    r"\sup": "sup", r"\inf": "inf", r"\min": "min", r"\max": "max",
    r"\int": "integral", r"\frac": "frac", r"\boxed": "",
    r"\begin{bmatrix}": "[", r"\end{bmatrix}": "]",
    r"\begin{aligned}": "", r"\end{aligned}": "",
    r"\text": "", r"\underbrace": "", r"\quad": "  ",
    r"\left": "", r"\right": "", r"\;": " ", r"\,": " ",
    r"\cdot": "*", r"\ldots": "...", r"\Vert": "||",
    r"\sum": "sum", r"\sqrt": "sqrt", r"\infty": "infinity",
    r"\forall": "for all", r"\mu": "mu", r"\theta": "theta",
    r"\partial": "partial", r"\nabla": "nabla", r"\approx": "~",
    r"\neq": "!=", r"\times": "x", r"\pm": "+/-",
    r"\mathrm": "", r"\operatorname": "", r"\mathcal": "",
    r"\dot": "d/dt ", r"\!": "",
}

INLINE_RE = re.compile(r"(\\\(.+?\\\)|\*\*.+?\*\*|`.+?`|\[[^\]]+\]\(https?://[^)]+\))")


def safe_text(value: str) -> str:
    """Avoid Unicode super/subscript glyphs in ReportLab output."""
    replacements = {
        "⁰": "^0", "¹": "^1", "²": "^2", "³": "^3", "⁴": "^4",
        "⁵": "^5", "⁶": "^6", "⁷": "^7", "⁸": "^8", "⁹": "^9",
        "₀": "_0", "₁": "_1", "₂": "_2", "₃": "_3", "₄": "_4",
        "₅": "_5", "₆": "_6", "₇": "_7", "₈": "_8", "₉": "_9",
    }
    for src, dst in replacements.items():
        value = value.replace(src, dst)
    return value


def linearize_latex(text: str) -> str:
    value = " ".join(line.strip() for line in text.splitlines())
    for src, dst in sorted(LATEX_MAP.items(), key=lambda item: len(item[0]), reverse=True):
        value = value.replace(src, dst)
    value = value.replace(r"\\", "; ")
    for _ in range(4):
        value = re.sub(r"frac\{([^{}]+)\}\{([^{}]+)\}", r"(\1)/(\2)", value)
    value = re.sub(r"\{([^{}]+)\}", r"\1", value)
    value = value.replace(r"\{", "{").replace(r"\}", "}")
    value = re.sub(r"\\[A-Za-z]+", "", value)
    return safe_text(re.sub(r"\s+", " ", value).strip())


def inline_markup(text: str) -> str:
    out: list[str] = []
    pos = 0
    for match in INLINE_RE.finditer(text):
        out.append(escape(safe_text(text[pos:match.start()])))
        token = match.group(0)
        if token.startswith(r"\("):
            out.append(escape(linearize_latex(token[2:-2])))
        elif token.startswith("**"):
            out.append(f"<b>{escape(safe_text(token[2:-2]))}</b>")
        elif token.startswith("`"):
            out.append(f'<font color="#0B2545">{escape(safe_text(token[1:-1]))}</font>')
        else:
            m = re.match(r"\[([^\]]+)\]\((https?://[^)]+)\)", token)
            out.append(f'<link href="{escape(m.group(2), quote=True)}" color="#2E74B5">{escape(safe_text(m.group(1)))}</link>')
        pos = match.end()
    out.append(escape(safe_text(text[pos:])))
    return "".join(out)


def parse_table(lines: list[str], start: int) -> tuple[list[list[str]], int]:
    rows: list[list[str]] = []
    idx = start
    while idx < len(lines) and lines[idx].strip().startswith("|"):
        rows.append([c.strip() for c in lines[idx].strip().strip("|").split("|")])
        idx += 1
    if len(rows) >= 2 and all(re.fullmatch(r":?-{3,}:?", c.replace(" ", "")) for c in rows[1]):
        rows.pop(1)
    return rows, idx


def column_widths(rows: list[list[str]], available: float) -> list[float]:
    n = max(len(r) for r in rows)
    scores = []
    for col in range(n):
        longest = max((len(r[col]) if col < len(r) else 0) for r in rows)
        scores.append(max(8, min(longest, 44)))
    total = sum(scores)
    widths = [available * score / total for score in scores]
    return widths


def build_styles():
    pdfmetrics.registerFont(TTFont(FONT, str(FONT_PATH)))
    pdfmetrics.registerFontFamily(FONT, normal=FONT, bold=FONT, italic=FONT, boldItalic=FONT)
    styles = getSampleStyleSheet()
    styles.add(ParagraphStyle(
        name="TCBody", fontName=FONT, fontSize=9.4, leading=13.2,
        textColor=INK, spaceAfter=5.5, allowWidows=0, allowOrphans=0,
    ))
    styles.add(ParagraphStyle(
        name="TCH1", parent=styles["TCBody"], fontSize=15.2, leading=19,
        textColor=BLUE, spaceBefore=14, spaceAfter=7, keepWithNext=True,
    ))
    styles.add(ParagraphStyle(
        name="TCH2", parent=styles["TCBody"], fontSize=12.3, leading=16,
        textColor=BLUE, spaceBefore=10, spaceAfter=5, keepWithNext=True,
    ))
    styles.add(ParagraphStyle(
        name="TCH3", parent=styles["TCBody"], fontSize=10.8, leading=14,
        textColor=DARK_BLUE, spaceBefore=8, spaceAfter=4, keepWithNext=True,
    ))
    styles.add(ParagraphStyle(
        name="TCQuote", parent=styles["TCBody"], leftIndent=12, rightIndent=8,
        borderColor=BLUE, borderWidth=0.8, borderPadding=6, backColor=LIGHT_FILL,
        spaceBefore=3, spaceAfter=7,
    ))
    styles.add(ParagraphStyle(
        name="TCEquation", parent=styles["TCBody"], alignment=TA_CENTER,
        fontSize=9.2, leading=12, leftIndent=12, rightIndent=12,
        borderPadding=5, backColor=LIGHT_FILL, spaceBefore=3, spaceAfter=6,
    ))
    styles.add(ParagraphStyle(
        name="TCTable", parent=styles["TCBody"], fontSize=7.35, leading=9.6,
        spaceAfter=0,
    ))
    styles.add(ParagraphStyle(
        name="TCCode", parent=styles["TCBody"], fontSize=7.4, leading=9.2,
        leftIndent=8, rightIndent=8, borderColor=RULE, borderWidth=0.5,
        borderPadding=6, backColor=LIGHT_FILL, spaceBefore=3, spaceAfter=7,
    ))
    return styles


class FrameworkDocTemplate(BaseDocTemplate):
    def __init__(self, filename: str, **kwargs):
        super().__init__(filename, pagesize=letter, leftMargin=0.72 * inch,
                         rightMargin=0.72 * inch, topMargin=0.72 * inch,
                         bottomMargin=0.68 * inch, **kwargs)
        frame = Frame(self.leftMargin, self.bottomMargin, self.width, self.height,
                      leftPadding=0, rightPadding=0, topPadding=0, bottomPadding=0)
        self.addPageTemplates(PageTemplate(id="main", frames=[frame], onPage=self.draw_page))

    def draw_page(self, canvas, doc):
        canvas.saveState()
        if doc.page > 1:
            canvas.setFont(FONT, 7.4)
            canvas.setFillColor(MUTED)
            canvas.drawString(self.leftMargin, letter[1] - 0.39 * inch,
                              "TCPS-PA v4.1  ·  单次运行通用实时性—物理安全双向诊断框架")
            canvas.setStrokeColor(RULE)
            canvas.setLineWidth(0.4)
            canvas.line(self.leftMargin, letter[1] - 0.44 * inch,
                        letter[0] - self.rightMargin, letter[1] - 0.44 * inch)
            canvas.drawRightString(letter[0] - self.rightMargin, 0.35 * inch,
                                   f"第 {doc.page} 页")
        canvas.restoreState()


def cover(styles) -> list:
    return [
        Spacer(1, 1.15 * inch),
        Paragraph('<font color="#2E74B5"><b>METHOD REFERENCE · TCPS-PA v4.1</b></font>',
                  ParagraphStyle("CoverKick", parent=styles["TCBody"], alignment=TA_CENTER, fontSize=10, leading=13)),
        Spacer(1, 0.42 * inch),
        Paragraph("自动驾驶单次运行通用实时性—物理安全<br/>双向诊断框架",
                  ParagraphStyle("CoverTitle", parent=styles["TCBody"], alignment=TA_CENTER,
                                 fontSize=24, leading=31, textColor=INK, spaceAfter=16)),
        Paragraph("Four Event Times · Observability · Typed Component Contracts",
                  ParagraphStyle("CoverSub", parent=styles["TCBody"], alignment=TA_CENTER,
                                 fontSize=10.7, leading=14, textColor=DARK_BLUE)),
        Spacer(1, 0.22 * inch),
        HRFlowable(width="42%", thickness=0.7, color=RULE, spaceBefore=4, spaceAfter=18, hAlign="CENTER"),
        Paragraph("<b>适用系统</b>　CARLA 0.9.15 + Apollo 10.0.0 + Bridge",
                  ParagraphStyle("CoverMeta1", parent=styles["TCBody"], alignment=TA_CENTER, fontSize=9.4, leading=14)),
        Paragraph("<b>分析范围</b>　一次 run 内的事件响应、多源一致性、连续控制环、双层契约与物理损失",
                  ParagraphStyle("CoverMeta2", parent=styles["TCBody"], alignment=TA_CENTER, fontSize=9.4, leading=14)),
        Paragraph("<b>证据约束</b>　Observed / Requirement / Model / Retrospective 严格分离",
                  ParagraphStyle("CoverMeta3", parent=styles["TCBody"], alignment=TA_CENTER, fontSize=9.4, leading=14)),
        Paragraph("<b>版本日期</b>　2026-08-13",
                  ParagraphStyle("CoverMeta4", parent=styles["TCBody"], alignment=TA_CENTER, fontSize=9.4, leading=14)),
        Spacer(1, 0.35 * inch),
        Paragraph('<font color="#9B1C1C"><b>本方法只针对单次运行；不包含多 run 组间比较方法</b></font>',
                  ParagraphStyle("CoverNote", parent=styles["TCBody"], alignment=TA_CENTER, fontSize=9.2)),
        PageBreak(),
    ]


def markdown_story(text: str, styles, available: float) -> list:
    lines = text.splitlines()
    idx = 0
    while idx < len(lines) and lines[idx].strip() != "---":
        idx += 1
    idx += 1
    story: list = []
    in_code = False
    code_lines: list[str] = []
    in_math = False
    math_lines: list[str] = []

    while idx < len(lines):
        raw = lines[idx]
        stripped = raw.strip()
        if in_code:
            if stripped.startswith("```"):
                # Preformatted's internal split can reserve the rest of a page
                # after a continuation.  Explicit chunks keep following
                # headings in normal flow and avoid nearly blank pages.
                for start in range(0, len(code_lines), 32):
                    chunk = code_lines[start:start + 32]
                    story.append(Preformatted(safe_text("\n".join(chunk)), styles["TCCode"], maxLineLength=110))
                code_lines, in_code = [], False
            else:
                code_lines.append(raw)
            idx += 1
            continue
        if in_math:
            if stripped == r"\]":
                story.append(Paragraph(escape(linearize_latex("\n".join(math_lines))), styles["TCEquation"]))
                math_lines, in_math = [], False
            else:
                math_lines.append(raw)
            idx += 1
            continue
        if stripped.startswith("```"):
            in_code = True
            idx += 1
            continue
        if stripped == r"\[":
            in_math = True
            idx += 1
            continue
        if not stripped:
            idx += 1
            continue
        if stripped == "---":
            story.append(HRFlowable(width="100%", thickness=0.45, color=RULE,
                                    spaceBefore=4, spaceAfter=7))
            idx += 1
            continue
        if stripped.startswith("|") and idx + 1 < len(lines) and lines[idx + 1].strip().startswith("|"):
            rows, idx = parse_table(lines, idx)
            ncols = max(len(r) for r in rows)
            data = []
            for row_idx, row in enumerate(rows):
                cells = []
                for col in range(ncols):
                    value = row[col] if col < len(row) else ""
                    cells.append(Paragraph(inline_markup(value), styles["TCTable"]))
                data.append(cells)
            table = Table(data, colWidths=column_widths(rows, available), repeatRows=1,
                          hAlign="LEFT", splitByRow=1)
            table.setStyle(TableStyle([
                ("BACKGROUND", (0, 0), (-1, 0), TABLE_FILL),
                ("TEXTCOLOR", (0, 0), (-1, 0), INK),
                ("GRID", (0, 0), (-1, -1), 0.35, RULE),
                ("VALIGN", (0, 0), (-1, -1), "TOP"),
                ("LEFTPADDING", (0, 0), (-1, -1), 4),
                ("RIGHTPADDING", (0, 0), (-1, -1), 4),
                ("TOPPADDING", (0, 0), (-1, -1), 3),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
            ]))
            story.extend([table, Spacer(1, 6)])
            continue
        m = re.match(r"^(#{1,4})\s+(.+)$", stripped)
        if m:
            source_level = len(m.group(1))
            if source_level == 1:
                idx += 1
                continue
            style = styles[{2: "TCH1", 3: "TCH2", 4: "TCH3"}.get(source_level, "TCH3")]
            story.append(Paragraph(inline_markup(m.group(2)), style))
            idx += 1
            continue
        if stripped.startswith(">"):
            quote = []
            while idx < len(lines) and lines[idx].strip().startswith(">"):
                quote.append(lines[idx].strip()[1:].strip())
                idx += 1
            story.append(Paragraph(inline_markup(" ".join(quote)), styles["TCQuote"]))
            continue
        m = re.match(r"^[-*]\s+(.+)$", stripped)
        if m:
            story.append(Paragraph(inline_markup(m.group(1)), styles["TCBody"], bulletText="•"))
            idx += 1
            continue
        m = re.match(r"^(\d+)\.\s+(.+)$", stripped)
        if m:
            story.append(Paragraph(inline_markup(m.group(2)), styles["TCBody"], bulletText=f"{m.group(1)}."))
            idx += 1
            continue

        para = [stripped]
        idx += 1
        while idx < len(lines):
            nxt = lines[idx].strip()
            if (not nxt or nxt == "---" or nxt == r"\[" or nxt.startswith("```")
                    or nxt.startswith("#") or nxt.startswith("|") or nxt.startswith(">")
                    or re.match(r"^[-*]\s+", nxt) or re.match(r"^\d+\.\s+", nxt)):
                break
            para.append(nxt)
            idx += 1
        story.append(Paragraph(inline_markup(" ".join(para)), styles["TCBody"]))
    return story


def main() -> None:
    if not FONT_PATH.exists():
        raise FileNotFoundError(FONT_PATH)
    styles = build_styles()
    doc = FrameworkDocTemplate(
        str(OUTPUT),
        title="自动驾驶单次运行通用实时性—物理安全双向诊断框架",
        author="TCPS-PA Method Working Draft",
        subject="Single-run event-and-loop temporal correctness and physical safety diagnosis",
        keywords="Apollo, CARLA, temporal coherence, closed loop, architectural contract, dynamic deadline",
    )
    story = cover(styles)
    story.extend(markdown_story(SOURCE.read_text(encoding="utf-8"), styles, doc.width))
    doc.build(story)
    print(OUTPUT)


if __name__ == "__main__":
    main()
