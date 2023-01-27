
import pandas as pd

df = pd.DataFrame(columns=['filename', 'id', 'center_x', 'center_y', 'w', 'h', 'category', 'relOrientation'])


def newDetection(**kwargs):
    pd.concat(kwargs, ignore_index=True, inplace=True)