import cadquery as cq
from Helpers import show

def move_to_center(cqObject,shape):
    '''
    Moves the origin of the current Workplane to the center of a given geometry object
    '''

    #transfrom to workplane local coords
    shape_center = shape.Center().sub(cqObject.plane.origin)
    #project ont plane using dot product
    x_offset = shape_center.dot(cqObject.plane.xDir)
    y_offset = shape_center.dot(cqObject.plane.yDir)

    return cqObject.center(x_offset,y_offset)

main_plate_size_y = 67
main_plate_size_x = 50.
main_plate_thickness = 10.

wing_size_x = 10.
wing_size_y = 10.

bridge_depth = 35.

support_depth = 18.

cutout_depth = 15.
cutout_rad = 8.
cutout_offset = 2.

extruder_hole_spacing = 50.

m4_predrill = 3.7
m3_predrill = 2.5
m3_cbore = 5.

mounting_hole_spacing = 28.

aux_hole_depth = 6.
aux_hole_spacing = 5.
aux_hole_N = 2

#main plate
res = cq.Workplane('front').box(main_plate_size_x,
                                main_plate_size_y,
                                main_plate_thickness)

def add_wing(obj, sign=1):
    obj = obj.workplane()\
            .hLine(sign*wing_size_x)\
            .vLine(-wing_size_y)\
            .line(-sign*wing_size_x, -2*wing_size_y)\
            .close().extrude(main_plate_thickness)
    return obj

#add wings
res = res.faces('<Z').vertices('>XY')
res = add_wing(res)  # add right wing

#store sides of the plate for further reuse

face_right = res.faces('>X[1]').val()
face_left  = res.faces('>X[-2]').val()

res = res.faces('<Z').vertices('>Y').vertices('<X')
res = add_wing(res, -1)  # add left wing

wp = res.faces('>Z')
e = wp.edges('>Y')

bridge_length = e.val().Length()

res = e.vertices('<X'). \
        workplane(). \
        hLine(bridge_length). \
        vLine(-10). \
        hLine(-bridge_length). \
        close().extrude(bridge_depth)

faces = res.faces('>Z[1]')
edge = faces.edges('>Y')
res = move_to_center(faces.workplane(),edge.val()).\
                     transformed(rotate=(0, 90, 0))

res = res.vLine(-support_depth).\
    line(-support_depth,support_depth).\
    close()

res = res.extrude(main_plate_size_x/2,both=True,clean=True)

face = res.faces('>Y')

res = move_to_center(face.workplane(),face.edges('>Z').val())

def make_slot(obj,depth=None):
    obj = obj.moveTo(cutout_rad,-cutout_depth).\
        threePointArc((0,-cutout_depth-cutout_rad),
                      (-cutout_rad,-cutout_depth)).\
        vLineTo(0).hLineTo(cutout_rad).close()

    if depth is None:
        obj = obj.cutThruAll()
    else:
        obj = obj.cutBlind(depth)

    return obj

res = make_slot(res,None)

cutout_rad += cutout_offset
res = make_slot(res.end().end(),-main_plate_thickness/2)

res = res.end().moveTo(0,0).\
      pushPoints([(-extruder_hole_spacing/2,-cutout_depth),
            (extruder_hole_spacing/2,-cutout_depth)]).\
      hole(m4_predrill)



cutout_rad += 3*cutout_offset
res = make_slot(res.end().moveTo(0,0).workplane(offset=-main_plate_thickness))

#add reinforcement holes
cutout_rad -= 2*cutout_offset
res = res.faces('>Z').workplane().\
          pushPoints([(-cutout_rad,-main_plate_thickness/4),
                      (cutout_rad,-main_plate_thickness/4)]).\
          hole(m3_predrill)

#add aux holes on the front face
res = res.moveTo(-main_plate_size_x/2.,0).workplane().rarray(aux_hole_spacing,1,aux_hole_N,1).hole(m3_predrill,depth=aux_hole_depth)
res = res.moveTo(main_plate_size_x,0).workplane().rarray(aux_hole_spacing,1,aux_hole_N,1).hole(m3_predrill,depth=aux_hole_depth)

#make a hexagonal cutout
res = res.faces('>Z[1]')
res = res.workplane(offset=bridge_depth). \
      transformed(rotate=(0,0,90)). \
      polygon(6,30).cutThruAll()

#make 4 mouting holes with cbores
res = res.end().moveTo(0,0). \
      rect(mounting_hole_spacing,
           mounting_hole_spacing,forConstruction=True)

res = res.vertices(). \
          cboreHole(m3_predrill,
                    m3_cbore,
                    bridge_depth+m3_cbore/2)

#make cutout and holes for mounting of the fan
res = res.transformed(rotate=(0,0,45)). \
      rect(35,35).cutBlind(-bridge_depth).end(). \
      rect(25,25,forConstruction=True).vertices().hole(m3_predrill)

def make_aux_holes(workplane,holes_span,N_hole_groups = 3):
    res = workplane.moveTo(-holes_span/2).\
         workplane().rarray(aux_hole_spacing,1,aux_hole_N,1).hole(m3_predrill,depth=aux_hole_depth)
    for i in range(N_hole_groups-1):
         res = res.moveTo(holes_span/(N_hole_groups-1.)).workplane().rarray(aux_hole_spacing,1,aux_hole_N,1).hole(m3_predrill,depth=aux_hole_depth)

    return res

#make aux holes at the bottom
res = res.faces('<Y').workplane()
res = make_aux_holes(res,main_plate_size_x*2/3.,3)

#make aux holes at the side (@overhang)
res = res.faces('<X').workplane().transformed((90,0,0))
res = make_aux_holes(res,main_plate_size_x*2/3.,3)

res = res.faces('>X').workplane().transformed((90,0,0))
res = make_aux_holes(res,main_plate_size_x*2/3.,3)

#make aux holes at the side (@main plate)
res = res.faces('|X').edges('<Y').edges('>X')

res = res.workplane()
res = move_to_center(res,face_right)

res = res.transformed((90,0,0))

hole_sep = 0.5*face_right.Area()/main_plate_thickness

res = make_aux_holes(res,hole_sep,2)

#make aux holes at the side (@main plate)
res = res.faces('|X').edges('<Y').edges('<X')

res = res.workplane()
res = move_to_center(res,face_left)

res = res.transformed((0,180,0))

hole_sep = 0.5*face_right.Area()/main_plate_thickness

res = make_aux_holes(res,hole_sep,2)

#show the result
show(res)
