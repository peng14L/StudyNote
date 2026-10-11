from tqdm import tqdm
import time

def create_tqdm_bar(iterable, desc, disable=False):
    return tqdm(iterable, desc=desc, disable=disable,
                unit='batch', dynamic_ncols=True)

# pbar = create_tqdm_bar(range(100), desc='Downing')
# for i in pbar:
#     time.sleep(0.05)

items = ['a', 'b', 'c', 'd']
for item in tqdm(items, desc='Processing'):
    time.sleep(0.5)