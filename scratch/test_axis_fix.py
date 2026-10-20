import sys
from PyQt6.QtWidgets import QApplication
import pyqtgraph as pg

app = QApplication(sys.argv)

class NonNegativeAxisItem(pg.AxisItem):
    """AxisItem that suppresses negative numbers and subnormal jitter."""
    def tickStrings(self, values, scale, spacing):
        strings = []
        for val in values:
            if val <= 0 or abs(val) < 1e-6:
                strings.append("0")
            elif val >= 10000:
                strings.append(f"{val:.0f}")
            elif val.is_integer():
                strings.append(f"{int(val)}")
            elif spacing < 1.0:
                strings.append(f"{val:.1f}".rstrip("0").rstrip("."))
            else:
                strings.append(f"{val:.0f}")
        return strings

class ProfilePlotWidget(pg.PlotWidget):
    def __init__(self, *args, **kwargs):
        if "axisItems" not in kwargs:
            kwargs["axisItems"] = {
                "left": NonNegativeAxisItem(orientation="left"),
                "bottom": NonNegativeAxisItem(orientation="bottom"),
            }
        super().__init__(*args, **kwargs)

vb = pg.ViewBox()
vb.disableAutoRange()
vb.setLimits(xMin=0.0, yMin=0.0, minXRange=5.0, minYRange=10.0)
vb.setXRange(0.0, 1000.0, padding=0)
vb.setYRange(0.0, 100.0, padding=0)

plot = ProfilePlotWidget(viewBox=vb)
plot.show()
app.processEvents()

vr = vb.viewRange()
print("View range:", vr)
left_ax = plot.getAxis("left")
ticks = left_ax.tickStrings([0.0, -5e-306, 20.0, 40.0, 60.0, 80.0, 100.0], 1, 20)
print("Left axis tick strings:", ticks)
assert vr[0][0] >= 0.0, "x min is negative"
assert vr[1][0] >= 0.0, "y min is negative"
assert all("-" not in t for t in ticks), f"Found negative in ticks: {ticks}"
print("TEST PASSED SUCCESSFULLY")
