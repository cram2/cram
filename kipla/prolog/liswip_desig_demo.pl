
liswip_desig_demo([['LocationDesigs: ', LocDesigs],
                   ['ObjectDesigs: ', ObjDesigs],
                   ['ObjDesigVals: ', ObjDesigVals]]) :-
  % Get all location desigs
  setof(LocDesig, desig_class(LocDesig, location), LocDesigs),
  setof(ObjDesig, desig_class(ObjDesig, object), ObjDesigs),
  setof(ObjDesigVal, (member(O, ObjDesigs),
                      desig_value(O, ObjDesigVal)),
        ObjDesigVals).
