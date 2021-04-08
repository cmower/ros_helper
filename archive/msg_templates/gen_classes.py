#!/usr/bin/env python

def unique(ls):
    out = []
    for el in ls:
        if el not in out:
            out.append(el)
    return out

def get_required_packages(pkgt):
    pkgs = []
    for msgt in pkgt
        for attt in msgt:
            pkgs.append(attt.attrib['package'])
    return unique(pkgs)
