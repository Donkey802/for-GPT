import maya.cmds as cmds
import math

# ==================================
# Utility Functions
# ==================================
def resolve_transform(node):
    if not cmds.objExists(node):
        raise RuntimeError(f'ノードが存在しません: {node}')
    current = node
    while True:
        if cmds.nodeType(current) in ('transform', 'joint'):
            return current
        parents = cmds.listRelatives(current, parent=True, fullPath=True) or []
        if not parents:
            break
        current = parents[0]
    raise RuntimeError(f'transform/joint ノードが見つかりません: {node}')


# ==================================
# ChainSelection
# ==================================
class ChainSelection:
    def __init__(self, nodes):
        self.nodes = nodes

    @classmethod
    def from_ui(cls, fields):
        nodes = []
        for f in fields:
            name = cmds.textFieldButtonGrp(f, query=True, text=True)
            if not name:
                raise RuntimeError('すべてのスロットにコントローラを設定してください。')
            nodes.append(resolve_transform(name))
        return cls(nodes)

    def validate(self, min_count=3):
        if len(self.nodes) < min_count:
            raise RuntimeError(f'コントローラは最低{min_count}つ必要です。')
        for n in self.nodes:
            if not cmds.objExists(n) or cmds.nodeType(n) not in ('transform', 'joint'):
                raise RuntimeError(f'有効な transform/joint ではありません: {n}')


# ==================================
# PVCalculator
# ==================================
class PVCalculator:
    @staticmethod
    def compute(j1, j2, j3, offset):
        p1 = cmds.xform(j1, q=True, ws=True, t=True)
        p2 = cmds.xform(j2, q=True, ws=True, t=True)
        p3 = cmds.xform(j3, q=True, ws=True, t=True)
        v1 = [p2[i] - p1[i] for i in range(3)]
        v2 = [p3[i] - p2[i] for i in range(3)]
        normal = [
            v1[1]*v2[2] - v1[2]*v2[1],
            v1[2]*v2[0] - v1[0]*v2[2],
            v1[0]*v2[1] - v1[1]*v2[0]
        ]
        length = math.sqrt(sum(n*n for n in normal))
        if length < 1e-6:
            return [p2[0], p2[1] + offset, p2[2]]
        return [p2[i] + normal[i]/length*offset for i in range(3)]


# ==================================
# IKRigBuilder
# ==================================
class IKRigBuilder:
    def __init__(self, chain, pv_offset, upv_locator=None):
        self.chain = chain
        self.pv_offset = pv_offset
        self.upv_locator = upv_locator
        self._own_upv_locator = False
        self.temp_joints = []
        self.ikh = None
        self.ik_ctrl = None
        self.pv_ctrl = None
        self.cons = []
        self.ik_to_fk_orient_cons = []
        self.start = cmds.playbackOptions(q=True, min=True)
        self.end = cmds.playbackOptions(q=True, max=True)

    def create_ik(self):
        # 1. Joint chain from FK controllers（選択依存を排除して明示的に親子付け）
        prev = None
        for i, ctrl in enumerate(self.chain.nodes):
            pos = cmds.xform(ctrl, q=True, ws=True, t=True)
            rot = cmds.xform(ctrl, q=True, ws=True, rotation=True)
            name = f"tempIK_jnt_{i+1}"

            cmds.select(clear=True)

            if prev:
                j = cmds.joint(name=name)
                cmds.parent(j, prev)
            else:
                j = cmds.joint(name=name)

            cmds.xform(j, ws=True, t=pos)
            cmds.xform(j, ws=True, rotation=rot)
            cmds.makeIdentity(j, apply=True, rotate=True)

            self.temp_joints.append(j)
            prev = j

        cmds.select(clear=True)

        # 追加: tempIK_jnt_1 を FK 根元に常時追従させる（位置＋向き）
        fk_root = resolve_transform(self.chain.nodes[0])
        temp_root_jnt = self.temp_joints[0]

        pc = cmds.pointConstraint(fk_root, temp_root_jnt, maintainOffset=False)
        oc = cmds.orientConstraint(fk_root, temp_root_jnt, maintainOffset=False)
        if pc:
            self.cons.append(pc[0] if isinstance(pc, (list, tuple)) else pc)
        if oc:
            self.cons.append(oc[0] if isinstance(oc, (list, tuple)) else oc)

        # 2. IK Handle
        root, eff = self.temp_joints[0], self.temp_joints[-1]
        self.ikh, _ = cmds.ikHandle(name='tempIK_handle', startJoint=root, endEffector=eff, solver='ikRPsolver')
        self.cons.append(self.ikh)

        # 3. IK Controller
        self.ik_ctrl = cmds.circle(name='tempIK_ctrl', normal=[1, 0, 0])[0]
        cmds.matchTransform(self.ik_ctrl, eff)
        pc2 = cmds.parentConstraint(self.ik_ctrl, self.ikh, mo=True)
        self.cons.append(pc2[0] if isinstance(pc2, (list, tuple)) else pc2)

        # 4. Pole Vector
        self.pv_ctrl = cmds.spaceLocator(name='tempIK_pv')[0]
        if self.upv_locator and cmds.objExists(self.upv_locator):
            pv_pos = cmds.xform(self.upv_locator, q=True, ws=True, t=True)
        else:
            pv_pos = PVCalculator.compute(*self.temp_joints[:3], self.pv_offset)
        cmds.xform(self.pv_ctrl, ws=True, t=pv_pos)
        pvc = cmds.poleVectorConstraint(self.pv_ctrl, self.ikh)
        self.cons.append(pvc[0] if isinstance(pvc, (list, tuple)) else pvc)

        # 5. IK→FK の orient constraint（bake 後に再構築される）
        for ctrl_node, jnt in zip(self.chain.nodes, self.temp_joints):
            oc2 = cmds.orientConstraint(jnt, ctrl_node, mo=True)[0]
            self.ik_to_fk_orient_cons.append(oc2)
            self.cons.append(oc2)

        # 6. FK アニメーションを IK に転送
        self._transfer_fk_animation_to_ik()

    def _transfer_fk_animation_to_ik(self):
        start, end = self.start, self.end
        end_fk = self.chain.nodes[-1]
        mid_fk = self.chain.nodes[1]

        for name, obj in [('末端FK', end_fk), ('IK', self.ik_ctrl), ('中間FK', mid_fk), ('PV', self.pv_ctrl)]:
            if not cmds.objExists(obj):
                raise RuntimeError(f'{name} "{obj}" が存在しません')

        temp_permanent_cons_to_delete = []
        for oc in self.ik_to_fk_orient_cons:
            if cmds.objExists(oc):
                temp_permanent_cons_to_delete.append(oc)

        if temp_permanent_cons_to_delete:
            cmds.delete(temp_permanent_cons_to_delete)
            self.cons = [c for c in self.cons if c not in temp_permanent_cons_to_delete]
            self.ik_to_fk_orient_cons = []

        temp_bake_cons = []
        try:
            temp_bake_cons.append(cmds.parentConstraint(end_fk, self.ik_ctrl, mo=False)[0])
            temp_bake_cons.append(cmds.parentConstraint(mid_fk, self.pv_ctrl, mo=False)[0])
            cmds.bakeResults([self.ik_ctrl, self.pv_ctrl], t=(start, end), sampleBy=1, simulation=True,
                             preserveOutsideKeys=True, at=['tx', 'ty', 'tz', 'rx', 'ry', 'rz'])
        finally:
            for c in temp_bake_cons:
                if cmds.objExists(c):
                    cmds.delete(c)

        for ctrl_node, jnt in zip(self.chain.nodes, self.temp_joints):
            oc3 = cmds.orientConstraint(jnt, ctrl_node, mo=True)[0]
            self.ik_to_fk_orient_cons.append(oc3)
            self.cons.append(oc3)

    def bake_and_cleanup(self):
        cmds.bakeResults(self.chain.nodes, t=(self.start, self.end), sampleBy=1, simulation=True,
                         at=['rotateX', 'rotateY', 'rotateZ'])
        self._delete_temp()

    def cleanup(self):
        self._delete_temp()

    def _delete_temp(self):
        for c in list(self.cons):
            if cmds.objExists(c):
                cmds.delete(c)

        for oc in list(self.ik_to_fk_orient_cons):
            if cmds.objExists(oc):
                cmds.delete(oc)

        for obj in (self.ikh, self.ik_ctrl, self.pv_ctrl):
            if obj and cmds.objExists(obj):
                cmds.delete(obj)
        for j in self.temp_joints:
            if cmds.objExists(j):
                cmds.delete(j)
        if self._own_upv_locator and self.upv_locator and cmds.objExists(self.upv_locator):
            cmds.delete(self.upv_locator)

        self.temp_joints = []
        self.cons = []
        self.ik_to_fk_orient_cons = []
        self.ikh = self.ik_ctrl = self.pv_ctrl = None


# ==================================
# UI: FKIKSwitcherUI
# ==================================
class FKIKSwitcherUI:
    def __init__(self):
        self.window_name = "FKIK_Switcher_UI"
        self.controller_fields = []
        self.num_controllers = 3
        self.current_builder = None
        self.upv_field = None
        self.upv_locator = None

    def show(self):
        if cmds.window(self.window_name, exists=True):
            cmds.deleteUI(self.window_name)
        self.window_name = cmds.window(self.window_name, title="FK⇔IK Switcher", widthHeight=(450, 380))
        main = cmds.columnLayout(adjustableColumn=True, parent=self.window_name)

        cmds.text(label="Number of FK Controllers:", align="left", parent=main)
        self.num_ctrl_field = cmds.intSliderGrp(field=True, min=3, max=10, value=self.num_controllers,
                                                fieldMinValue=3, fieldMaxValue=100, step=1,
                                                columnWidth3=(120, 50, 200), label="Count:",
                                                changeCommand=self._update_controller_fields, parent=main)
        self.ctrl_field_layout = cmds.columnLayout(adjustableColumn=True, parent=main)
        self._create_controller_fields(self.num_controllers)

        cmds.separator(height=10, style='in', parent=main)
        cmds.text(label="アップベクター設定", align="left", parent=main)
        self.upv_field = cmds.textFieldButtonGrp(label="アップベクター元FKコントローラ:",
                                                buttonLabel="Set",
                                                columnWidth=[(1, 150), (2, 200), (3, 50)],
                                                parent=main)
        cmds.textFieldButtonGrp(self.upv_field, edit=True, buttonCommand=self._set_upv_field)
        cmds.button(label="アップベクター仮生成", command=self._create_upv_locator, width=140, parent=main)

        cmds.separator(height=10, style='in', parent=main)
        self.pv_offset_field = cmds.floatFieldGrp(numberOfFields=1, label="Pole Vector Offset:",
                                                  value1=1.0, parent=main)

        cmds.separator(height=10, style='none', parent=main)
        cmds.rowLayout(numberOfColumns=3, columnWidth3=(140, 140, 140), adjustableColumn=3, parent=main)
        cmds.button(label="Generate IK", command=self._on_generate_ik, width=130)
        cmds.button(label="Bake & Delete IK", command=self._on_bake_and_delete, width=130)
        cmds.button(label="Delete IK Only", command=self._on_delete_ik, width=130)

        cmds.showWindow(self.window_name)

    def _create_controller_fields(self, count):
        for f in self.controller_fields:
            if cmds.control(f, exists=True):
                cmds.deleteUI(f)
        self.controller_fields = []
        for i in range(count):
            f = cmds.textFieldButtonGrp(label=f"FK Controller {i+1}:", buttonLabel="Select",
                                        columnWidth=[(1, 100), (2, 200), (3, 50)],
                                        parent=self.ctrl_field_layout)
            cmds.textFieldButtonGrp(f, edit=True, buttonCommand=lambda fld=f: self._load_selected_controller(fld))
            self.controller_fields.append(f)

    def _update_controller_fields(self, new_count):
        self.num_controllers = int(new_count)
        self._create_controller_fields(self.num_controllers)

    def _load_selected_controller(self, field):
        sel = cmds.ls(selection=True)
        if sel:
            tx = resolve_transform(sel[0])
            cmds.textFieldButtonGrp(field, edit=True, text=tx)
        else:
            cmds.warning("No object selected to load into the field.")

    def _set_upv_field(self, *args):
        sel = cmds.ls(selection=True)
        if sel:
            tx = resolve_transform(sel[0])
            cmds.textFieldButtonGrp(self.upv_field, edit=True, text=tx)
        else:
            cmds.warning("No object selected for up-vector source.")

    def _create_upv_locator(self, *args):
        if self.upv_locator and cmds.objExists(self.upv_locator):
            cmds.delete(self.upv_locator)
        src = cmds.textFieldButtonGrp(self.upv_field, query=True, text=True)
        if not src or not cmds.objExists(src):
            cmds.error("アップベクター元FKコントローラを設定してください。")
            return
        pos = cmds.xform(src, q=True, ws=True, t=True)
        rot = cmds.xform(src, q=True, ws=True, rotation=True)
        self.upv_locator = cmds.spaceLocator(name='temp_upVector_locator#')[0]
        cmds.xform(self.upv_locator, ws=True, t=pos)
        cmds.xform(self.upv_locator, ws=True, rotation=rot)

    def _on_generate_ik(self, *args):
        try:
            chain = ChainSelection.from_ui(self.controller_fields)
            chain.validate()
            pv = cmds.floatFieldGrp(self.pv_offset_field, query=True, value1=True)
            self.current_builder = IKRigBuilder(chain, pv, self.upv_locator)
            self.current_builder.create_ik()
        except Exception as e:
            cmds.error(f"Error: {e}")

    def _on_bake_and_delete(self, *args):
        if not self.current_builder:
            cmds.error('まずGenerate IKを実行してください')
            return
        try:
            self.current_builder.bake_and_cleanup()
            self.current_builder = None
        except Exception as e:
            cmds.error(f"Error: {e}")

    def _on_delete_ik(self, *args):
        if not self.current_builder:
            cmds.error('まずGenerate IKを実行してください')
            return
        try:
            self.current_builder.cleanup()
            self.current_builder = None
        except Exception as e:
            cmds.error(f"Error: {e}")


# UI 起動
ui = FKIKSwitcherUI()
ui.show()
