import NemAll_Python_Geometry as AllplanGeo
import NemAll_Python_BaseElements as AllplanBaseElements
import NemAll_Python_BasisElements as AllplanBasisElements
import NemAll_Python_Utility as AllplanUtil
import GeometryValidate as GeometryValidate
from HandleDirection import HandleDirection
from HandleProperties import HandleProperties


def check_allplan_version(build_ele, version):
    del build_ele
    del version
    return True


def create_element(build_ele, doc):
    element = BridgeBeam(doc)
    return element.create(build_ele)


def move_handle(build_ele, handle_prop, input_pnt, doc):
    build_ele.change_property(handle_prop, input_pnt)
    return create_element(build_ele, doc)


class BridgeBeam:
    def __init__(self, doc):
        self.model_ele_list = []
        self.handle_list = []
        self.document = doc

    def create(self, build_ele):
        self.top_part(build_ele)
        self.create_handles(build_ele)
        # self.ref(build_ele);
        return (self.model_ele_list, self.handle_list)

    def bottom_part(self, build_ele):
        brep = AllplanGeo.BRep3D.CreateCuboid(
            AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(0, 0, 0),
                                       AllplanGeo.Vector3D(1, 0, 0),
                                       AllplanGeo.Vector3D(0, 0, 1)),
            build_ele.BottomPartWidth.value,
            build_ele.GeneralLength.value,
            build_ele.BottomPartHeight.value)

        brep_inter = AllplanGeo.BRep3D.CreateCuboid(
            AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(0, 0, 0),
                                       AllplanGeo.Vector3D(1, 0, 0),
                                       AllplanGeo.Vector3D(0, 0, 1)),
            build_ele.BottomPartWidth.value,
            build_ele.GeneralLength.value,
            build_ele.BottomPartHeight.value)

        bottom_part_upper_slice = build_ele.BottomPartUpperSliceLength.value
        bottom_part_lower_slice = build_ele.BottomPartLowerSliceLength.value

        if bottom_part_upper_slice > 0:
            edges = AllplanUtil.VecSizeTList()
            edges.append(1)
            edges.append(3)

            err, brep = AllplanGeo.ChamferCalculus.Calculate(
                brep, edges, bottom_part_upper_slice, False)

            if not GeometryValidate.polyhedron(err):
                return

        if bottom_part_lower_slice > 0:
            edges2 = AllplanUtil.VecSizeTList()
            edges2.append(8)
            edges2.append(10)

            err, brep_inter = AllplanGeo.ChamferCalculus.Calculate(
                brep_inter, edges2, bottom_part_lower_slice, False)

            if not GeometryValidate.polyhedron(err):
                return

        err, done_part = AllplanGeo.MakeIntersection(brep, brep_inter)

        return done_part

    def central_part(self, build_ele):
        brep = AllplanGeo.BRep3D.CreateCuboid(
            AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(build_ele.BottomPartWidth.value / 2 - build_ele.CentralPartWidth.value / 2, 0, build_ele.BottomPartHeight.value),
                                       AllplanGeo.Vector3D(1, 0, 0),
                                       AllplanGeo.Vector3D(0, 0, 1)),
            build_ele.CentralPartWidth.value,
            build_ele.GeneralLength.value,
            build_ele.CentralPartHeight.value)

        hole1 = AllplanGeo.BRep3D.CreateCylinder(
            AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(build_ele.BottomPartUpperSliceLength.value, build_ele.GeneralLength.value / 8, build_ele.BottomPartHeight.value + build_ele.CentralPartHeight.value / 2),
                                       AllplanGeo.Vector3D(0, 0, 1),
                                       AllplanGeo.Vector3D(1, 0, 0)),
            build_ele.Diameter.value / 2, build_ele.CentralPartWidth.value)

        hole2 = AllplanGeo.BRep3D.CreateCylinder(
            AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(build_ele.BottomPartUpperSliceLength.value, build_ele.GeneralLength.value - build_ele.GeneralLength.value / 8, build_ele.BottomPartHeight.value + build_ele.CentralPartHeight.value / 2),
                                       AllplanGeo.Vector3D(0, 0, 1),
                                       AllplanGeo.Vector3D(1, 0, 0)),
            build_ele.Diameter.value / 2, build_ele.CentralPartWidth.value)

        err, brep = AllplanGeo.MakeSubtraction(brep, hole1)
        err, brep = AllplanGeo.MakeSubtraction(brep, hole2)

        err, done_part = AllplanGeo.MakeUnion(
            brep, self.bottom_part(build_ele))
        return done_part

    def top_part(self, build_ele):
        brep = AllplanGeo.BRep3D.CreateCuboid(
            AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(0 - (build_ele.TopPartWidth.value - build_ele.BottomPartWidth.value) / 2, 0, build_ele.BottomPartHeight.value + build_ele.CentralPartHeight.value),
                                       AllplanGeo.Vector3D(1, 0, 0),
                                       AllplanGeo.Vector3D(0, 0, 1)),
            build_ele.TopPartWidth.value,
            build_ele.GeneralLength.value,
            build_ele.TopPartHeight.value)

        brep_plate = AllplanGeo.BRep3D.CreateCuboid(
            AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(build_ele.PlateIndent.value - (build_ele.TopPartWidth.value - build_ele.BottomPartWidth.value) / 2, 0, build_ele.BottomPartHeight.value + build_ele.CentralPartHeight.value + build_ele.TopPartHeight.value),
                                       AllplanGeo.Vector3D(1, 0, 0),
                                       AllplanGeo.Vector3D(0, 0, 1)),
            build_ele.TopPartWidth.value - build_ele.PlateIndent.value*2,
            build_ele.GeneralLength.value,
            build_ele.PlateHeight.value)

        com_prop = AllplanBaseElements.CommonProperties()
        com_prop.GetGlobalProperties()
        com_prop.Pen = 1
        com_prop.Color = 1

        top_part_lower_slice = build_ele.TopPartLowerSlice.value

        if top_part_lower_slice > 0:
            edges2 = AllplanUtil.VecSizeTList()
            edges2.append(8)
            edges2.append(10)

            err, brep = AllplanGeo.ChamferCalculus.Calculate(
                brep, edges2, top_part_lower_slice, False)

            if not GeometryValidate.polyhedron(err):
                return

        err, done_part = AllplanGeo.MakeUnion(
            brep, self.central_part(build_ele))
        err, done_part = AllplanGeo.MakeUnion(done_part, brep_plate)
        self.model_ele_list.append(
            AllplanBasisElements.ModelElement3D(com_prop, done_part))

    def create_handles(self, build_ele):
        origin = AllplanGeo.Point3D(
            build_ele.BottomPartWidth.value / 2, build_ele.GeneralLength.value, build_ele.CentralPartHeight.value + build_ele.BottomPartHeight.value)
        origin2 = AllplanGeo.Point3D(
            build_ele.BottomPartWidth.value / 2, 0, build_ele.BottomPartHeight.value / 2)
        origin3 = AllplanGeo.Point3D(
            0, build_ele.GeneralLength.value, (build_ele.BottomPartHeight.value - build_ele.BottomPartUpperSliceLength.value) / 2)
        origin4 = AllplanGeo.Point3D(
            0 - (build_ele.TopPartWidth.value - build_ele.BottomPartWidth.value) / 2, build_ele.GeneralLength.value, build_ele.CentralPartHeight.value + build_ele.BottomPartHeight.value + build_ele.TopPartLowerSlice.value)
        origin5 = AllplanGeo.Point3D(
            build_ele.BottomPartWidth.value / 2, build_ele.GeneralLength.value, build_ele.CentralPartHeight.value + build_ele.BottomPartHeight.value - build_ele.BottomPartHeight.value / 4)
        origin6 = AllplanGeo.Point3D(
            build_ele.BottomPartWidth.value / 2, build_ele.GeneralLength.value, build_ele.CentralPartHeight.value + build_ele.BottomPartHeight.value + build_ele.TopPartHeight.value)
        origin7 = AllplanGeo.Point3D(
            build_ele.BottomPartWidth.value / 2, build_ele.GeneralLength.value, 0)
        origin8 = AllplanGeo.Point3D(
            build_ele.BottomPartWidth.value / 2 - build_ele.CentralPartWidth.value / 2, build_ele.GeneralLength.value, build_ele.CentralPartHeight.value / 2 + build_ele.BottomPartHeight.value)

        self.handle_list.append(
            HandleProperties("CentralPartHeight",
                             AllplanGeo.Point3D(origin.X,
                                                origin.Y,
                                                origin.Z),
                             AllplanGeo.Point3D(origin.X,
                                                origin.Y,
                                                origin.Z - build_ele.CentralPartHeight.value),
                             [("CentralPartHeight", HandleDirection.z_dir)],
                             HandleDirection.z_dir,
                             False))

        self.handle_list.append(
            HandleProperties("GeneralLength",
                             AllplanGeo.Point3D(origin2.X,
                                                origin2.Y + build_ele.GeneralLength.value,
                                                origin2.Z),
                             AllplanGeo.Point3D(origin2.X,
                                                origin2.Y,
                                                origin2.Z),
                             [("GeneralLength", HandleDirection.y_dir)],
                             HandleDirection.y_dir,
                             False))

        self.handle_list.append(
            HandleProperties("BottomPartWidth", AllplanGeo.Point3D(origin3.X + build_ele.BottomPartWidth.value, origin3.Y, origin3.Z),
                             AllplanGeo.Point3D(
                                 origin3.X, origin3.Y, origin3.Z),
                             [("BottomPartWidth", HandleDirection.x_dir)],
                             HandleDirection.x_dir,
                             False))

        self.handle_list.append(
            HandleProperties("TopPartWidth",
                             AllplanGeo.Point3D(origin4.X + build_ele.TopPartWidth.value,
                                                origin4.Y,
                                                origin4.Z),
                             AllplanGeo.Point3D(origin4.X,
                                                origin4.Y,
                                                origin4.Z),
                             [("TopPartWidth", HandleDirection.x_dir)],
                             HandleDirection.x_dir,
                             False))

        self.handle_list.append(
            HandleProperties("TopPartHeight",
                             AllplanGeo.Point3D(origin5.X,
                                                origin5.Y,
                                                origin5.Z + build_ele.TopPartHeight.value),
                             AllplanGeo.Point3D(origin5.X,
                                                origin5.Y,
                                                origin5.Z),
                             [("TopPartHeight", HandleDirection.z_dir)],
                             HandleDirection.z_dir,
                             False))

        self.handle_list.append(
            HandleProperties("PlateHeight",
                             AllplanGeo.Point3D(origin6.X,
                                                origin6.Y,
                                                origin6.Z + build_ele.PlateHeight.value),
                             AllplanGeo.Point3D(origin6.X,
                                                origin6.Y,
                                                origin6.Z),
                             [("PlateHeight", HandleDirection.z_dir)],
                             HandleDirection.z_dir,
                             False))

        self.handle_list.append(
            HandleProperties("BottomPartHeight",
                             AllplanGeo.Point3D(origin7.X,
                                                origin7.Y,
                                                origin7.Z + build_ele.BottomPartHeight.value),
                             AllplanGeo.Point3D(origin7.X,
                                                origin7.Y,
                                                origin7.Z),
                             [("BottomPartHeight", HandleDirection.z_dir)],
                             HandleDirection.z_dir,
                             False))

        self.handle_list.append(
            HandleProperties("CentralPartWidth",
                             AllplanGeo.Point3D(origin8.X + build_ele.CentralPartWidth.value,
                                                origin8.Y,
                                                origin8.Z),
                             AllplanGeo.Point3D(origin8.X,
                                                origin8.Y,
                                                origin8.Z),
                             [("CentralPartWidth", HandleDirection.x_dir)],
                             HandleDirection.x_dir,
                             False))
