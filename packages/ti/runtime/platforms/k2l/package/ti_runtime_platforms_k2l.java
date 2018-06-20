/*
 *  Do not modify this file; it is automatically 
 *  generated and any modifications will be overwritten.
 *
 * @(#) xdc-A75
 */
import java.util.*;
import org.mozilla.javascript.*;
import xdc.services.intern.xsr.*;
import xdc.services.spec.Session;

public class ti_runtime_platforms_k2l
{
    static final String VERS = "@(#) xdc-A75\n";

    static final Proto.Elm $$T_Bool = Proto.Elm.newBool();
    static final Proto.Elm $$T_Num = Proto.Elm.newNum();
    static final Proto.Elm $$T_Str = Proto.Elm.newStr();
    static final Proto.Elm $$T_Obj = Proto.Elm.newObj();

    static final Proto.Fxn $$T_Met = new Proto.Fxn(null, null, 0, -1, false);
    static final Proto.Map $$T_Map = new Proto.Map($$T_Obj);
    static final Proto.Arr $$T_Vec = new Proto.Arr($$T_Obj);

    static final XScriptO $$DEFAULT = Value.DEFAULT;
    static final Object $$UNDEF = Undefined.instance;

    static final Proto.Obj $$Package = (Proto.Obj)Global.get("$$Package");
    static final Proto.Obj $$Module = (Proto.Obj)Global.get("$$Module");
    static final Proto.Obj $$Instance = (Proto.Obj)Global.get("$$Instance");
    static final Proto.Obj $$Params = (Proto.Obj)Global.get("$$Params");

    static final Object $$objFldGet = Global.get("$$objFldGet");
    static final Object $$objFldSet = Global.get("$$objFldSet");
    static final Object $$proxyGet = Global.get("$$proxyGet");
    static final Object $$proxySet = Global.get("$$proxySet");
    static final Object $$delegGet = Global.get("$$delegGet");
    static final Object $$delegSet = Global.get("$$delegSet");

    Scriptable xdcO;
    Session ses;
    Value.Obj om;

    boolean isROV;
    boolean isCFG;

    Proto.Obj pkgP;
    Value.Obj pkgV;

    ArrayList<Object> imports = new ArrayList<Object>();
    ArrayList<Object> loggables = new ArrayList<Object>();
    ArrayList<Object> mcfgs = new ArrayList<Object>();
    ArrayList<Object> icfgs = new ArrayList<Object>();
    ArrayList<String> inherits = new ArrayList<String>();
    ArrayList<Object> proxies = new ArrayList<Object>();
    ArrayList<Object> sizes = new ArrayList<Object>();
    ArrayList<Object> tdefs = new ArrayList<Object>();

    void $$IMPORTS()
    {
        Global.callFxn("loadPackage", xdcO, "xdc");
        Global.callFxn("loadPackage", xdcO, "xdc.corevers");
        Global.callFxn("loadPackage", xdcO, "xdc.platform");
        Global.callFxn("loadPackage", xdcO, "ti.platforms.generic");
    }

    void $$OBJECTS()
    {
        pkgP = (Proto.Obj)om.bind("ti.runtime.platforms.k2l.Package", new Proto.Obj());
        pkgV = (Value.Obj)om.bind("ti.runtime.platforms.k2l", new Value.Obj("ti.runtime.platforms.k2l", pkgP));
    }

    void Platform$$OBJECTS()
    {
        Proto.Obj po, spo;
        Value.Obj vo;

        po = (Proto.Obj)om.bind("ti.runtime.platforms.k2l.Platform.Module", new Proto.Obj());
        vo = (Value.Obj)om.bind("ti.runtime.platforms.k2l.Platform", new Value.Obj("ti.runtime.platforms.k2l.Platform", po));
        pkgV.bind("Platform", vo);
        // decls 
        om.bind("ti.runtime.platforms.k2l.Platform.Board", om.findStrict("xdc.platform.IPlatform.Board", "ti.runtime.platforms.k2l"));
        om.bind("ti.runtime.platforms.k2l.Platform.Memory", om.findStrict("xdc.platform.IPlatform.Memory", "ti.runtime.platforms.k2l"));
        // insts 
        Object insP = om.bind("ti.runtime.platforms.k2l.Platform.Instance", new Proto.Obj());
        po = (Proto.Obj)om.bind("ti.runtime.platforms.k2l.Platform$$Object", new Proto.Obj());
        om.bind("ti.runtime.platforms.k2l.Platform.Object", new Proto.Str(po, true));
        po = (Proto.Obj)om.bind("ti.runtime.platforms.k2l.Platform$$Params", new Proto.Obj());
        om.bind("ti.runtime.platforms.k2l.Platform.Params", new Proto.Str(po, true));
    }

    void Platform$$CONSTS()
    {
        // module Platform
    }

    void Platform$$CREATES()
    {
        Proto.Fxn fxn;
        StringBuilder sb;

        fxn = (Proto.Fxn)om.bind("ti.runtime.platforms.k2l.Platform$$create", new Proto.Fxn(om.findStrict("ti.runtime.platforms.k2l.Platform.Module", "ti.runtime.platforms.k2l"), om.findStrict("ti.runtime.platforms.k2l.Platform.Instance", "ti.runtime.platforms.k2l"), 2, 1, false));
                fxn.addArg(0, "name", $$T_Str, $$UNDEF);
                fxn.addArg(1, "__params", (Proto)om.findStrict("ti.runtime.platforms.k2l.Platform.Params", "ti.runtime.platforms.k2l"), Global.newObject());
        sb = new StringBuilder();
        sb.append("ti$runtime$platforms$k2l$Platform$$create = function( name, __params ) {\n");
            sb.append("var __mod = xdc.om['ti.runtime.platforms.k2l.Platform'];\n");
            sb.append("var __inst = xdc.om['ti.runtime.platforms.k2l.Platform.Instance'].$$make();\n");
            sb.append("__inst.$$bind('$package', xdc.om['ti.runtime.platforms.k2l']);\n");
            sb.append("__inst.$$bind('$index', __mod.$instances.length);\n");
            sb.append("__inst.$$bind('$category', 'Instance');\n");
            sb.append("__inst.$$bind('$args', {name:name});\n");
            sb.append("__inst.$$bind('$module', __mod);\n");
            sb.append("__mod.$instances.$add(__inst);\n");
            sb.append("__inst.externalMemoryMap = __mod.PARAMS.externalMemoryMap;\n");
            sb.append("__inst.customMemoryMap = __mod.PARAMS.customMemoryMap;\n");
            sb.append("__inst.renameMap = __mod.PARAMS.renameMap;\n");
            sb.append("__inst.dataMemory = __mod.PARAMS.dataMemory;\n");
            sb.append("__inst.codeMemory = __mod.PARAMS.codeMemory;\n");
            sb.append("__inst.stackMemory = __mod.PARAMS.stackMemory;\n");
            sb.append("__inst.sectMap = __mod.PARAMS.sectMap;\n");
            sb.append("__inst.peripherals = __mod.PARAMS.peripherals;\n");
            sb.append("for (var __p in __params) __inst[__p] = __params[__p];\n");
            sb.append("var save = xdc.om.$curpkg;\n");
            sb.append("xdc.om.$$bind('$curpkg', __mod.$package.$name);\n");
            sb.append("__mod.instance$meta$init.$fxn.apply(__inst, [name]);\n");
            sb.append("xdc.om.$$bind('$curpkg', save);\n");
            sb.append("__inst.$$bless();\n");
            sb.append("return __inst;\n");
        sb.append("}\n");
        Global.eval(sb.toString());
        fxn = (Proto.Fxn)om.bind("ti.runtime.platforms.k2l.Platform$$construct", new Proto.Fxn(om.findStrict("ti.runtime.platforms.k2l.Platform.Module", "ti.runtime.platforms.k2l"), null, 3, 1, false));
                fxn.addArg(0, "__obj", (Proto)om.findStrict("ti.runtime.platforms.k2l.Platform$$Object", "ti.runtime.platforms.k2l"), null);
                fxn.addArg(1, "name", $$T_Str, $$UNDEF);
                fxn.addArg(2, "__params", (Proto)om.findStrict("ti.runtime.platforms.k2l.Platform.Params", "ti.runtime.platforms.k2l"), Global.newObject());
        sb = new StringBuilder();
        sb.append("ti$runtime$platforms$k2l$Platform$$construct = function( __obj, name, __params ) {\n");
            sb.append("var __mod = xdc.om['ti.runtime.platforms.k2l.Platform'];\n");
            sb.append("var __inst = __obj;\n");
            sb.append("__inst.$$bind('$args', {name:name});\n");
            sb.append("__inst.$$bind('$module', __mod);\n");
            sb.append("__mod.$objects.$add(__inst);\n");
            sb.append("__inst.externalMemoryMap = __mod.PARAMS.externalMemoryMap;\n");
            sb.append("__inst.customMemoryMap = __mod.PARAMS.customMemoryMap;\n");
            sb.append("__inst.renameMap = __mod.PARAMS.renameMap;\n");
            sb.append("__inst.dataMemory = __mod.PARAMS.dataMemory;\n");
            sb.append("__inst.codeMemory = __mod.PARAMS.codeMemory;\n");
            sb.append("__inst.stackMemory = __mod.PARAMS.stackMemory;\n");
            sb.append("__inst.sectMap = __mod.PARAMS.sectMap;\n");
            sb.append("__inst.peripherals = __mod.PARAMS.peripherals;\n");
            sb.append("for (var __p in __params) __inst[__p] = __params[__p];\n");
            sb.append("__inst.$$bless();\n");
            sb.append("return null;\n");
        sb.append("}\n");
        Global.eval(sb.toString());
    }

    void Platform$$FUNCTIONS()
    {
        Proto.Fxn fxn;

    }

    void Platform$$SIZES()
    {
    }

    void Platform$$TYPES()
    {
        Scriptable cap;
        Proto.Obj po;
        Proto.Str ps;
        Proto.Typedef pt;
        Object fxn;

        cap = (Scriptable)Global.callFxn("loadCapsule", xdcO, "ti/runtime/platforms/k2l/Platform.xs");
        om.bind("ti.runtime.platforms.k2l.Platform$$capsule", cap);
        po = (Proto.Obj)om.findStrict("ti.runtime.platforms.k2l.Platform.Module", "ti.runtime.platforms.k2l");
        po.init("ti.runtime.platforms.k2l.Platform.Module", om.findStrict("xdc.platform.IPlatform.Module", "ti.runtime.platforms.k2l"));
                po.addFld("$hostonly", $$T_Num, 1, "r");
        po.addFld("CPU", (Proto)om.findStrict("ti.platforms.generic.Platform.Instance", "ti.runtime.platforms.k2l"), $$UNDEF, "wh");
                po.addFxn("create", (Proto.Fxn)om.findStrict("ti.runtime.platforms.k2l.Platform$$create", "ti.runtime.platforms.k2l"), Global.get("ti$runtime$platforms$k2l$Platform$$create"));
                po.addFxn("construct", (Proto.Fxn)om.findStrict("ti.runtime.platforms.k2l.Platform$$construct", "ti.runtime.platforms.k2l"), Global.get("ti$runtime$platforms$k2l$Platform$$construct"));
        fxn = Global.get(cap, "module$use");
        if (fxn != null) om.bind("ti.runtime.platforms.k2l.Platform$$module$use", true);
        if (fxn != null) po.addFxn("module$use", $$T_Met, fxn);
        fxn = Global.get(cap, "module$meta$init");
        if (fxn != null) om.bind("ti.runtime.platforms.k2l.Platform$$module$meta$init", true);
        if (fxn != null) po.addFxn("module$meta$init", $$T_Met, fxn);
        fxn = Global.get(cap, "instance$meta$init");
        if (fxn != null) om.bind("ti.runtime.platforms.k2l.Platform$$instance$meta$init", true);
        if (fxn != null) po.addFxn("instance$meta$init", $$T_Met, fxn);
        fxn = Global.get(cap, "module$validate");
        if (fxn != null) om.bind("ti.runtime.platforms.k2l.Platform$$module$validate", true);
        if (fxn != null) po.addFxn("module$validate", $$T_Met, fxn);
        po = (Proto.Obj)om.findStrict("ti.runtime.platforms.k2l.Platform.Instance", "ti.runtime.platforms.k2l");
        po.init("ti.runtime.platforms.k2l.Platform.Instance", om.findStrict("xdc.platform.IPlatform.Instance", "ti.runtime.platforms.k2l"));
                po.addFld("$hostonly", $$T_Num, 1, "r");
        po.addFld("codeMemory", $$T_Str, "L2SRAM", "wh");
        po.addFld("dataMemory", $$T_Str, "L2SRAM", "wh");
        po.addFld("stackMemory", $$T_Str, "L2SRAM", "wh");
        po.addFld("l2Mode", $$T_Str, "0k", "wh");
        po.addFld("l1PMode", $$T_Str, "0k", "wh");
        po.addFld("l1DMode", $$T_Str, "0k", "wh");
                fxn = Global.get(cap, "getCpuDataSheet");
                if (fxn != null) po.addFxn("getCpuDataSheet", (Proto.Fxn)om.findStrict("xdc.platform.IPlatform$$getCpuDataSheet", "ti.runtime.platforms.k2l"), fxn);
                fxn = Global.get(cap, "getCreateArgs");
                if (fxn != null) po.addFxn("getCreateArgs", (Proto.Fxn)om.findStrict("xdc.platform.IPlatform$$getCreateArgs", "ti.runtime.platforms.k2l"), fxn);
                fxn = Global.get(cap, "getExeContext");
                if (fxn != null) po.addFxn("getExeContext", (Proto.Fxn)om.findStrict("xdc.platform.IPlatform$$getExeContext", "ti.runtime.platforms.k2l"), fxn);
                fxn = Global.get(cap, "getExecCmd");
                if (fxn != null) po.addFxn("getExecCmd", (Proto.Fxn)om.findStrict("xdc.platform.IPlatform$$getExecCmd", "ti.runtime.platforms.k2l"), fxn);
                fxn = Global.get(cap, "getLinkTemplate");
                if (fxn != null) po.addFxn("getLinkTemplate", (Proto.Fxn)om.findStrict("xdc.platform.IPlatform$$getLinkTemplate", "ti.runtime.platforms.k2l"), fxn);
        po = (Proto.Obj)om.findStrict("ti.runtime.platforms.k2l.Platform$$Params", "ti.runtime.platforms.k2l");
        po.init("ti.runtime.platforms.k2l.Platform.Params", om.findStrict("xdc.platform.IPlatform$$Params", "ti.runtime.platforms.k2l"));
                po.addFld("$hostonly", $$T_Num, 1, "r");
        po.addFld("codeMemory", $$T_Str, "L2SRAM", "wh");
        po.addFld("dataMemory", $$T_Str, "L2SRAM", "wh");
        po.addFld("stackMemory", $$T_Str, "L2SRAM", "wh");
        po.addFld("l2Mode", $$T_Str, "0k", "wh");
        po.addFld("l1PMode", $$T_Str, "0k", "wh");
        po.addFld("l1DMode", $$T_Str, "0k", "wh");
        po = (Proto.Obj)om.findStrict("ti.runtime.platforms.k2l.Platform$$Object", "ti.runtime.platforms.k2l");
        po.init("ti.runtime.platforms.k2l.Platform.Object", om.findStrict("ti.runtime.platforms.k2l.Platform.Instance", "ti.runtime.platforms.k2l"));
                fxn = Global.get(cap, "getCpuDataSheet");
                if (fxn != null) po.addFxn("getCpuDataSheet", (Proto.Fxn)om.findStrict("xdc.platform.IPlatform$$getCpuDataSheet", "ti.runtime.platforms.k2l"), fxn);
                fxn = Global.get(cap, "getCreateArgs");
                if (fxn != null) po.addFxn("getCreateArgs", (Proto.Fxn)om.findStrict("xdc.platform.IPlatform$$getCreateArgs", "ti.runtime.platforms.k2l"), fxn);
                fxn = Global.get(cap, "getExeContext");
                if (fxn != null) po.addFxn("getExeContext", (Proto.Fxn)om.findStrict("xdc.platform.IPlatform$$getExeContext", "ti.runtime.platforms.k2l"), fxn);
                fxn = Global.get(cap, "getExecCmd");
                if (fxn != null) po.addFxn("getExecCmd", (Proto.Fxn)om.findStrict("xdc.platform.IPlatform$$getExecCmd", "ti.runtime.platforms.k2l"), fxn);
                fxn = Global.get(cap, "getLinkTemplate");
                if (fxn != null) po.addFxn("getLinkTemplate", (Proto.Fxn)om.findStrict("xdc.platform.IPlatform$$getLinkTemplate", "ti.runtime.platforms.k2l"), fxn);
    }

    void Platform$$ROV()
    {
    }

    void $$SINGLETONS()
    {
        pkgP.init("ti.runtime.platforms.k2l.Package", (Proto.Obj)om.findStrict("xdc.IPackage.Module", "ti.runtime.platforms.k2l"));
        pkgP.bind("$capsule", $$UNDEF);
        pkgV.init2(pkgP, "ti.runtime.platforms.k2l", Value.DEFAULT, false);
        pkgV.bind("$name", "ti.runtime.platforms.k2l");
        pkgV.bind("$category", "Package");
        pkgV.bind("$$qn", "ti.runtime.platforms.k2l.");
        pkgV.bind("$vers", Global.newArray());
        Value.Map atmap = (Value.Map)pkgV.getv("$attr");
        atmap.seal("length");
        imports.clear();
        pkgV.bind("$imports", imports);
        StringBuilder sb = new StringBuilder();
        sb.append("var pkg = xdc.om['ti.runtime.platforms.k2l'];\n");
        sb.append("if (pkg.$vers.length >= 3) {\n");
            sb.append("pkg.$vers.push(Packages.xdc.services.global.Vers.getDate(xdc.csd() + '/..'));\n");
        sb.append("}\n");
        sb.append("if ('ti.runtime.platforms.k2l$$stat$base' in xdc.om) {\n");
            sb.append("pkg.packageBase = xdc.om['ti.runtime.platforms.k2l$$stat$base'];\n");
            sb.append("pkg.packageRepository = xdc.om['ti.runtime.platforms.k2l$$stat$root'];\n");
        sb.append("}\n");
        sb.append("pkg.build.libraries = [\n");
        sb.append("];\n");
        sb.append("pkg.build.libDesc = [\n");
        sb.append("];\n");
        Global.eval(sb.toString());
    }

    void Platform$$SINGLETONS()
    {
        Proto.Obj po;
        Value.Obj vo;

        vo = (Value.Obj)om.findStrict("ti.runtime.platforms.k2l.Platform", "ti.runtime.platforms.k2l");
        po = (Proto.Obj)om.findStrict("ti.runtime.platforms.k2l.Platform.Module", "ti.runtime.platforms.k2l");
        vo.init2(po, "ti.runtime.platforms.k2l.Platform", $$DEFAULT, false);
        vo.bind("Module", po);
        vo.bind("$category", "Module");
        vo.bind("$capsule", om.findStrict("ti.runtime.platforms.k2l.Platform$$capsule", "ti.runtime.platforms.k2l"));
        vo.bind("Instance", om.findStrict("ti.runtime.platforms.k2l.Platform.Instance", "ti.runtime.platforms.k2l"));
        vo.bind("Params", om.findStrict("ti.runtime.platforms.k2l.Platform.Params", "ti.runtime.platforms.k2l"));
        vo.bind("PARAMS", ((Proto.Str)om.findStrict("ti.runtime.platforms.k2l.Platform.Params", "ti.runtime.platforms.k2l")).newInstance());
        vo.bind("$package", om.findStrict("ti.runtime.platforms.k2l", "ti.runtime.platforms.k2l"));
        tdefs.clear();
        proxies.clear();
        mcfgs.clear();
        icfgs.clear();
        inherits.clear();
        vo.bind("Board", om.findStrict("xdc.platform.IPlatform.Board", "ti.runtime.platforms.k2l"));
        tdefs.add(om.findStrict("xdc.platform.IPlatform.Board", "ti.runtime.platforms.k2l"));
        vo.bind("Memory", om.findStrict("xdc.platform.IPlatform.Memory", "ti.runtime.platforms.k2l"));
        tdefs.add(om.findStrict("xdc.platform.IPlatform.Memory", "ti.runtime.platforms.k2l"));
        vo.bind("MemoryMap", om.findStrict("xdc.platform.IPlatform.MemoryMap", "ti.runtime.platforms.k2l"));
        vo.bind("$$tdefs", Global.newArray(tdefs.toArray()));
        vo.bind("$$proxies", Global.newArray(proxies.toArray()));
        vo.bind("$$mcfgs", Global.newArray(mcfgs.toArray()));
        vo.bind("$$icfgs", Global.newArray(icfgs.toArray()));
        inherits.add("xdc.platform");
        vo.bind("$$inherits", Global.newArray(inherits.toArray()));
        ((Value.Arr)pkgV.getv("$modules")).add(vo);
        ((Value.Arr)om.findStrict("$modules", "ti.runtime.platforms.k2l")).add(vo);
        vo.bind("$$instflag", 1);
        vo.bind("$$iobjflag", 1);
        vo.bind("$$sizeflag", 1);
        vo.bind("$$dlgflag", 0);
        vo.bind("$$iflag", 1);
        vo.bind("$$romcfgs", "|");
        vo.bind("$$nortsflag", 0);
        Proto.Str ps = (Proto.Str)vo.find("Module_State");
        if (ps != null) vo.bind("$object", ps.newInstance());
        vo.bind("$$meta_iobj", om.has("ti.runtime.platforms.k2l.Platform$$instance$static$init", null) ? 1 : 0);
        vo.bind("$$fxntab", Global.newArray());
        vo.bind("$$logEvtCfgs", Global.newArray());
        vo.bind("$$errorDescCfgs", Global.newArray());
        vo.bind("$$assertDescCfgs", Global.newArray());
        Value.Map atmap = (Value.Map)vo.getv("$attr");
        atmap.seal("length");
        vo.bind("Object", om.findStrict("ti.runtime.platforms.k2l.Platform.Object", "ti.runtime.platforms.k2l"));
        pkgV.bind("Platform", vo);
        ((Value.Arr)pkgV.getv("$unitNames")).add("Platform");
    }

    void $$INITIALIZATION()
    {
        Value.Obj vo;

        if (isCFG) {
        }//isCFG
        Global.callFxn("module$meta$init", (Scriptable)om.findStrict("ti.runtime.platforms.k2l.Platform", "ti.runtime.platforms.k2l"));
        vo = (Value.Obj)om.findStrict("ti.runtime.platforms.k2l.Platform", "ti.runtime.platforms.k2l");
        Global.put(vo, "CPU", Global.callFxn("create", (Scriptable)om.find("ti.platforms.generic.Platform"), "CPU", Global.newObject("clockRate", 1000L, "catalogName", "ti.catalog.c6000", "deviceName", "Kepler", "customMemoryMap", Global.newArray(new Object[]{Global.newArray(new Object[]{"L1DSRAM", Global.newObject("name", "L1DSRAM", "base", 0x00f00000L, "len", 0x00008000L, "space", "data", "access", "RW")}), Global.newArray(new Object[]{"L1PSRAM", Global.newObject("name", "L1PSRAM", "base", 0x00e00000L, "len", 0x00008000L, "space", "code", "access", "RWX")}), Global.newArray(new Object[]{"L2SRAM", Global.newObject("name", "L2SRAM", "base", 0x00800000L, "len", 0x00100000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"LTE1_MSMC_SHARED_HEAP", Global.newObject("name", "LTE1_MSMC_SHARED_HEAP", "base", 0x0C000000L, "len", 0x00010000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"LTE1_MSMC_L2DP", Global.newObject("name", "LTE1_MSMC_L2DP", "base", 0x0C010000L, "len", 0x00010000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"LTE1_MSMC_L1SLAVE", Global.newObject("name", "LTE1_MSMC_L1SLAVE", "base", 0x0C020000L, "len", 0x00010000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"LTE1_MSMC_L1MASTER", Global.newObject("name", "LTE1_MSMC_L1MASTER", "base", 0x0C030000L, "len", 0x00010000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"LTE1_MSMC_L2SCHED", Global.newObject("name", "LTE1_MSMC_L2SCHED", "base", 0x0C040000L, "len", 0x00010000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"LTE2_MSMC_SHARED_HEAP", Global.newObject("name", "LTE2_MSMC_SHARED_HEAP", "base", 0x0C050000L, "len", 0x00010000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"LTE2_MSMC_L2DP", Global.newObject("name", "LTE2_MSMC_L2DP", "base", 0x0C060000L, "len", 0x00010000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"LTE2_MSMC_L1SLAVE", Global.newObject("name", "LTE2_MSMC_L1SLAVE", "base", 0x0C070000L, "len", 0x00010000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"LTE2_MSMC_L1MASTER", Global.newObject("name", "LTE2_MSMC_L1MASTER", "base", 0x0C080000L, "len", 0x00010000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"LTE2_MSMC_L2SCHED", Global.newObject("name", "LTE2_MSMC_L2SCHED", "base", 0x0C090000L, "len", 0x00010000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"DDR3_SYSLIB_ROOT_RSVD_MEM", Global.newObject("name", "DDR3_SYSLIB_ROOT_RSVD_MEM", "base", 0xA0000000L, "len", 0x00010000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"DDR3_SYSLIB_RESMGR_RSVD", Global.newObject("name", "DDR3_SYSLIB_RESMGR_RSVD", "base", 0xA0010000L, "len", 0x0000F000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"LTE1_DDR3_NAME_PROXY_RSVD", Global.newObject("name", "LTE1_DDR3_NAME_PROXY_RSVD", "base", 0xA001F000L, "len", 0x00001000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"LTE1_DDR3_NAMED_RESOURCE", Global.newObject("name", "LTE1_DDR3_NAMED_RESOURCE", "base", 0xA0020000L, "len", 0x00001000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"LTE1_DDR3_SHARED_HEAP", Global.newObject("name", "LTE1_DDR3_SHARED_HEAP", "base", 0xA0021000L, "len", 0x01000000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"LTE1_DDR3_L2DP", Global.newObject("name", "LTE1_DDR3_L2DP", "base", 0xA1021000L, "len", 0x01000000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"LTE1_DDR3_L1SLAVE", Global.newObject("name", "LTE1_DDR3_L1SLAVE", "base", 0xA2021000L, "len", 0x01000000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"LTE1_DDR3_L1MASTER", Global.newObject("name", "LTE1_DDR3_L1MASTER", "base", 0xA3021000L, "len", 0x01000000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"LTE1_DDR3_L2SCHED", Global.newObject("name", "LTE1_DDR3_L2SCHED", "base", 0xA4021000L, "len", 0x01000000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"LTE2_DDR3_NAME_PROXY_RSVD", Global.newObject("name", "LTE2_DDR3_NAME_PROXY_RSVD", "base", 0xB0000000L, "len", 0x00001000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"LTE2_DDR3_NAMED_RESOURCE", Global.newObject("name", "LTE2_DDR3_NAMED_RESOURCE", "base", 0xB0001000L, "len", 0x00001000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"LTE2_DDR3_SHARED_HEAP", Global.newObject("name", "LTE2_DDR3_SHARED_HEAP", "base", 0xB0002000L, "len", 0x01000000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"LTE2_DDR3_L2DP", Global.newObject("name", "LTE2_DDR3_L2DP", "base", 0xB1002000L, "len", 0x01000000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"LTE2_DDR3_L1SLAVE", Global.newObject("name", "LTE2_DDR3_L1SLAVE", "base", 0xB2002000L, "len", 0x01000000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"LTE2_DDR3_L1MASTER", Global.newObject("name", "LTE2_DDR3_L1MASTER", "base", 0xB3002000L, "len", 0x01000000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"LTE2_DDR3_L2SCHED", Global.newObject("name", "LTE2_DDR3_L2SCHED", "base", 0xB4002000L, "len", 0x01000000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"LTE3_DDR3_NAME_PROXY_RSVD", Global.newObject("name", "LTE3_DDR3_NAME_PROXY_RSVD", "base", 0xB5002000L, "len", 0x00001000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"LTE3_DDR3_NAMED_RESOURCE", Global.newObject("name", "LTE3_DDR3_NAMED_RESOURCE", "base", 0xB5003000L, "len", 0x00001000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"LTE4_DDR3_NAME_PROXY_RSVD", Global.newObject("name", "LTE4_DDR3_NAME_PROXY_RSVD", "base", 0xB5004000L, "len", 0x00001000L, "space", "code/data", "access", "RWX")}), Global.newArray(new Object[]{"LTE4_DDR3_NAMED_RESOURCE", Global.newObject("name", "LTE4_DDR3_NAMED_RESOURCE", "base", 0xB5005000L, "len", 0x00001000L, "space", "code/data", "access", "RWX")})}), "l2Mode", "0k", "l1PMode", "0k", "l1DMode", "0k")));
        Global.callFxn("init", pkgV);
        ((Value.Obj)om.getv("ti.runtime.platforms.k2l.Platform")).bless();
        ((Value.Arr)om.findStrict("$packages", "ti.runtime.platforms.k2l")).add(pkgV);
    }

    public void exec( Scriptable xdcO, Session ses )
    {
        this.xdcO = xdcO;
        this.ses = ses;
        om = (Value.Obj)xdcO.get("om", null);

        Object o = om.geto("$name");
        String s = o instanceof String ? (String)o : null;
        isCFG = s != null && s.equals("cfg");
        isROV = s != null && s.equals("rov");

        $$IMPORTS();
        $$OBJECTS();
        Platform$$OBJECTS();
        Platform$$CONSTS();
        Platform$$CREATES();
        Platform$$FUNCTIONS();
        Platform$$SIZES();
        Platform$$TYPES();
        if (isROV) {
            Platform$$ROV();
        }//isROV
        $$SINGLETONS();
        Platform$$SINGLETONS();
        $$INITIALIZATION();
    }
}
