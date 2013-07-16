import bpy
import mathutils
import math

bl_info = {
    "name": "blender2bullet",
    "author": "",
    "version": (1, 0),
    "blender": (2, 66, 0),
    "location": "",
    "description": "Export blender physics to bullet file",
    "warning": "",
    "wiki_url": "",
    "tracker_url": "",
    "category": "System"}


class CustomMenu(bpy.types.Menu):
    bl_label = "Bullet Export"
    bl_idname = "OBJECT_MT_bulletexport_menu"

    def draw(self, context):
        layout = self.layout

        layout.operator("wm.bulletexport", text="Blender physics to bullet file")


def draw_item(self, context):
    layout = self.layout
    layout.menu(CustomMenu.bl_idname)


class BulletExport(bpy.types.Operator):
    bl_idname = "wm.bulletexport"
    bl_label = "Blender physics to bullet file"

#    def rotate_all_objects(self, angle, axis):
#        rot = mathutils.Matrix.Rotation(angle, 4, axis)
#        for obj in bpy.data.objects:
#            if obj.parent is None:
#                obj.select = True
#                obj.matrix_world = rot * obj.matrix_world
#                bpy.ops.object.transform_apply(rotation=True)

    def changeValue(self,value1,value2):
        value  = value1
        value1 = value2
        value2 = -value

    def rotateRigidBody(self,obj):
        locationY = obj.location.y
        obj.location.y =  obj.location.z
        obj.location.z = -locationY
        if obj.rotation_mode == 'XYZ':
            obj.rotation_mode = 'XZY'
        elif obj.rotation_mode == 'XZY':
            obj.rotation_mode = 'XYZ'
        elif obj.rotation_mode == 'YXZ':
            obj.rotation_mode = 'ZXY'
        elif obj.rotation_mode == 'YZX':
            obj.rotation_mode = 'ZYX'
        elif obj.rotation_mode == 'ZXY':
            obj.rotation_mode = 'YXZ'
        elif obj.rotation_mode == 'ZYX':
            obj.rotation_mode = 'YZX'
        rotationY = obj.rotation_euler[1]
        obj.rotation_euler[1] = obj.rotation_euler[2]
        obj.rotation_euler[2] = -rotationY

        if hasattr(obj.data, 'vertices'):
            for vertex in obj.data.vertices:
                self.rotateVertex(vertex)

        if hasattr(obj, 'children'):
            for child in obj.children:
                self.rotateRigidBody(child)

        for const in obj.constraints:
            if const.name.startswith('cr_'):
                self.rotateConstraint(const)

    def rotateRigidBodyBack(self,obj):
        z = obj.location.z
        obj.location.z =  obj.location.y
        obj.location.y = -z
        if obj.rotation_mode == 'XYZ':
            obj.rotation_mode = 'XZY'
        elif obj.rotation_mode == 'XZY':
            obj.rotation_mode = 'XYZ'
        elif obj.rotation_mode == 'YXZ':
            obj.rotation_mode = 'ZXY'
        elif obj.rotation_mode == 'YZX':
            obj.rotation_mode = 'ZYX'
        elif obj.rotation_mode == 'ZXY':
            obj.rotation_mode = 'YXZ'
        elif obj.rotation_mode == 'ZYX':
            obj.rotation_mode = 'YZX'
        rotationZ = obj.rotation_euler[2]
        obj.rotation_euler[2] = obj.rotation_euler[1]
        obj.rotation_euler[1] = -rotationZ

        if hasattr(obj.data, 'vertices'):
            for vertex in obj.data.vertices:
                self.rotateVertexBack(vertex)

        if hasattr(obj, 'children'):
            for child in obj.children:
                self.rotateRigidBodyBack(child)

        for const in obj.constraints:
            if const.name.startswith('cr_'):
                self.rotateConstraintBack(const)

    def rotateConstraint(self,const):
        pivotY = const.pivot_y
        const.pivot_y =  const.pivot_z
        const.pivot_z = -pivotY

        eul = mathutils.Euler( ( const.axis_x, const.axis_y, const.axis_z ), 'XYZ' )

        eulY = eul.y
        eul.y =  eul.z
        eul.z = -eulY
        eul.order = 'XZY'

        quat = eul.to_quaternion()
        eul2 = quat.to_euler( 'XYZ' )

        const.axis_x = eul2.x
        const.axis_y = eul2.y
        const.axis_z = eul2.z

        limitMinY = const.limit_min_y
        const.limit_min_y = const.limit_min_z
        const.limit_min_z = -limitMinY
        limitMaxY = const.limit_max_y
        const.limit_max_y = const.limit_max_z
        const.limit_max_z = -limitMaxY
        limitAngleMinY = const.limit_angle_min_y
        const.limit_angle_min_y = const.limit_angle_min_z
        const.limit_angle_min_z = -limitAngleMinY
        limitAngleMaxY = const.limit_angle_max_y
        const.limit_angle_max_y = const.limit_angle_max_z
        const.limit_angle_max_z = -limitAngleMaxY

        limitMinZ = const.limit_min_z
        const.limit_min_z = const.limit_max_z
        const.limit_max_z = limitMinZ
        limitAngleMinZ = const.limit_angle_min_z
        const.limit_angle_min_z = const.limit_angle_max_z
        const.limit_angle_max_z = limitAngleMinZ

    def rotateConstraintBack(self,const):
        pivotZ = const.pivot_z
        const.pivot_z =  const.pivot_y
        const.pivot_y = -pivotZ

        eul = mathutils.Euler( ( const.axis_x, const.axis_y, const.axis_z ), 'XYZ' )

        eulZ = eul.z
        eul.z =  eul.y
        eul.y = -eulZ
        eul.order = 'XZY'

        quat = eul.to_quaternion()
        eul2 = quat.to_euler( 'XYZ' )

        const.axis_x = eul2.x
        const.axis_y = eul2.y
        const.axis_z = eul2.z

        limitMinZ = const.limit_min_z
        const.limit_min_z = const.limit_min_y
        const.limit_min_y = -limitMinZ
        limitMaxZ = const.limit_max_z
        const.limit_max_z = const.limit_max_y
        const.limit_max_y = -limitMaxZ
        limitAngleMinZ = const.limit_angle_min_z
        const.limit_angle_min_z = const.limit_angle_min_y
        const.limit_angle_min_y = -limitAngleMinZ
        limitAngleMaxZ = const.limit_angle_max_z
        const.limit_angle_max_z = const.limit_angle_max_y
        const.limit_angle_max_y = -limitAngleMaxZ

        limitMinY = const.limit_min_y
        const.limit_min_y = const.limit_max_y
        const.limit_max_y = limitMinY
        limitAngleMinY = const.limit_angle_min_y
        const.limit_angle_min_y = const.limit_angle_max_y
        const.limit_angle_max_y = limitAngleMinY

    def rotateVertex(self,vertex):
        posY = vertex.co.y
        vertex.co.y =  vertex.co.z
        vertex.co.z = -posY

    def rotateVertexBack(self,vertex):
        posZ = vertex.co.z
        vertex.co.z =  vertex.co.y
        vertex.co.y = -posZ

    def deselectAll(self):
        for obj in bpy.data.objects:
            obj.select = False

    def clearParent(self):
        self.deselectAll()
        for obj in bpy.data.objects:
            if obj.name.startswith('rb_'):
                obj.select = True
        bpy.ops.object.parent_clear(type='CLEAR_KEEP_TRANSFORM')
        self.deselectAll()

    def rotateObjects(self):
        for obj in bpy.data.objects:
            if obj.name.startswith('rb_'):
                self.rotateRigidBody(obj)
            if obj.name.startswith('sb_'):
                self.rotateRigidBody(obj)

    def rotateObjectsBack(self):
        for obj in bpy.data.objects:
            if obj.name.startswith('rb_'):
                self.rotateRigidBodyBack(obj)
            if obj.name.startswith('sb_'):
                self.rotateRigidBodyBack(obj)

    def blender2bullet(self):

#        self.rotate_all_objects(-math.pi/2.0, 'X')
#        self.clearParent()
#        self.rotateObjects()
#        self.clear_all_parent()

        bpy.data.texts.new(name="SAVING_SCRIPT_PYSCRIPT") 
        txt = bpy.data.texts['SAVING_SCRIPT_PYSCRIPT']
        txt.write('''
import os
import bge
import bpy
import datetime

t = datetime.datetime.now().strftime("%Y.%m.%d.%H.%M.%S")
bulletfn = os.path.splitext(bpy.data.filepath)[0] + "." + t + ".bullet"
bge.constraints.exportBulletFile(bulletfn)
bge.logic.endGame()
''')

        ob = bpy.context.active_object
        bpy.ops.logic.sensor_add(type='ALWAYS')
        sen = ob.game.sensors[-1]
        sen.name = 'BULLET_EXPORT_SENSOR'

        bpy.ops.logic.controller_add()
        cont = ob.game.controllers[-1]
        cont.name = 'BULLET_EXPORT_PYTHON'
        cont.type = 'PYTHON'
        cont = ob.game.controllers[-1]
        cont.link(sen)
        cont.states = 1
        cont.mode = 'SCRIPT'
        cont.text = bpy.data.texts['SAVING_SCRIPT_PYSCRIPT']

        bpy.ops.view3d.game_start()

        bpy.ops.logic.sensor_remove(sensor = sen.name)
        bpy.ops.logic.controller_remove(controller = cont.name)

        bpy.data.texts.remove(txt)

#        self.rotate_all_objects(math.pi/2.0, 'X')
#        self.rotateObjectsBack()


    def execute(self, context):
        self.blender2bullet()
        return {'FINISHED'}


def register():
    bpy.utils.register_class(BulletExport)
    bpy.utils.register_class(CustomMenu)
    bpy.types.INFO_HT_header.append(draw_item)


def unregister():
    bpy.utils.unregister_class(BulletExport)
    bpy.utils.unregister_class(CustomMenu)
    bpy.types.INFO_HT_header.remove(draw_item)

if __name__ == "__main__":
    register()
    bpy.ops.wm.call_menu(name=CustomMenu.bl_idname)
