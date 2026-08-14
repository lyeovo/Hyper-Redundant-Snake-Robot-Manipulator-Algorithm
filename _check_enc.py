# -*- coding: utf-8 -*-
import os, glob
p = os.path.join(os.getcwd(), '_qa_analysis.m')
data = open(p, 'rb').read()
print('len', len(data), 'first bytes', data[:20].hex())
bad = [(i, b) for i, b in enumerate(data) if b > 127]
print('non-ascii bytes:', len(bad))
print('positions:', [x[0] for x in bad[:30]])
# try decode
for enc in ['utf-8', 'gbk', 'latin-1']:
    try:
        data.decode(enc)
        print('decodes as', enc)
    except Exception as e:
        print(enc, 'FAIL', e)
