// g++ -o rsrcc rsrcc.cpp -I /usr/lib/llvm-14/include/ -L /usr/lib/llvm-14/lib/ -lclang

#include "rsrcc.h"

#include <unordered_map>

static const std::string esp = "1";
static const std::string ebp = "2";
static const std::string eax = "3";

static constexpr int RESERVED_REGS = 3;

static constexpr int stackBegin = 4096;
static constexpr int stackSize = 4096;
static constexpr int MAX_REGS = 31;

static const std::string ENTRY_POINT = "main";

class Register {
    static int nextReg;
    int reg;

   public:
    Register() : reg(nextReg--) {}
    Register(const Register& other) : reg(other.reg) {}
    // Register(int reg) : reg(reg) {}
    Register& operator=(const Register& other)  = delete;
    Register& operator=(const Register&& other) = delete;
    operator int() const { return reg; }
    ~Register() { nextReg++; }
};

int Register::nextReg = MAX_REGS;

struct SymTabEntry {
    int offset;  //  negative offset from ebp for locals, posotive from ebp for params, and next available offset for params for func (ugly I know, TODO fix)
    // param1 
    // param2
    // param3
    // return address
    // saved previous ebp  <- ebp
    // local1
    // local2
    // local3 <- esp
    
    Register reg;

    enum class Kind { local, param, func, invalid } kind;

    std::string label;

    SymTabEntry() : reg(), offset(-1), kind(Kind::invalid) {}
};

static std::unordered_map<std::string, SymTabEntry> symTab;

static int labelNum = 0;

std::ostream& operator<<(std::ostream& stream, const CXString& str) {
    stream << clang_getCString(str);
    clang_disposeString(str);
    return stream;
}

std::ostream& operator<<(std::ostream& stream, const CXCursor& cursor) {
    stream << clang_getCursorKindSpelling(clang_getCursorKind(cursor)) << " <";
    switch (cursor.kind) {
        case CXCursor_UnaryOperator:
            stream << getUnaryOperator(cursor);
            break;
        case CXCursor_BinaryOperator:
            stream << getBinaryOperator(cursor);
            break;
        case CXCursor_IntegerLiteral:
            stream << getIntegerLiteral(cursor);
            break;
        case CXCursor_VarDecl:
        case CXCursor_ParmDecl:
        case CXCursor_DeclRefExpr:
        case CXCursor_UnexposedExpr:
            stream << getSymbolName(cursor);
            break;
        default:
            stream << clang_getCursorSpelling(cursor);
    }
    stream << ">";
    return stream;
}

void emit(std::string code) { std::cout << code << std::endl; }

std::vector<CXCursor> getChildren(CXCursor cursor) {
    std::vector<CXCursor> children;
    clang_visitChildren(
        cursor,
        [](CXCursor cursor, CXCursor parent, CXClientData clientData) {
            std::vector<CXCursor>* children = (std::vector<CXCursor>*)clientData;
            children->push_back(cursor);
            return CXChildVisit_Continue;
        },
        &children);
    return children;
}

std::vector<CXCursor> getDescendants(CXCursor cursor) {
    std::vector<CXCursor> descendants;
    clang_visitChildren(
        cursor,
        [](CXCursor cursor, CXCursor parent, CXClientData clientData) {
            std::vector<CXCursor>* descendants = (std::vector<CXCursor>*)clientData;
            descendants->push_back(cursor);
            return CXChildVisit_Recurse;
        },
        &descendants);
    return descendants;
}

std::vector<CXCursor> getParams(CXCursor cursor) {
    std::vector<CXCursor> params;
    for (auto child : getChildren(cursor)) {
        if (clang_getCursorKind(child) == CXCursor_ParmDecl) {
            params.push_back(child);
        }
    }
    return params;
}

std::vector<CXCursor> getArgs(CXCursor cursor) {
    auto children = getChildren(cursor);
    std::vector<CXCursor> args(children.begin() + 1, children.end());
    return args;
}

std::vector<CXCursor> getBody(CXCursor cursor) {
    std::vector<CXCursor> body;
    for (auto child : getChildren(cursor)) {
        if (clang_getCursorKind(child) != CXCursor_ParmDecl) {
            body.push_back(child);
        }
    }
    return body;
}

std::string getUnaryOperator(CXCursor cursor) {
    CXSourceRange range = clang_getCursorExtent(cursor);
    CXToken* tokens = nullptr;
    unsigned int numTokens = 0;
    CXTranslationUnit TU = clang_Cursor_getTranslationUnit(cursor);
    clang_tokenize(TU, range, &tokens, &numTokens);
    if (numTokens > 0) {
        CXString tokenSpelling = clang_getTokenSpelling(TU, tokens[0]);
        std::string op = clang_getCString(tokenSpelling);
        clang_disposeString(tokenSpelling);
        clang_disposeTokens(TU, tokens, numTokens);
        return op;
    }
    clang_disposeTokens(TU, tokens, numTokens);
    return "";
}

std::string getBinaryOperator(CXCursor cursor) {
    CXSourceRange range = clang_getCursorExtent(cursor);
    CXToken* tokens = nullptr;
    unsigned int numTokens = 0;
    CXTranslationUnit TU = clang_Cursor_getTranslationUnit(cursor);
    clang_tokenize(TU, range, &tokens, &numTokens);
    if (numTokens > 1) {
        CXString tokenSpelling = clang_getTokenSpelling(TU, tokens[1]);
        std::string op = clang_getCString(tokenSpelling);
        clang_disposeString(tokenSpelling);
        clang_disposeTokens(TU, tokens, numTokens);
        return op;
    }
    clang_disposeTokens(TU, tokens, numTokens);
    return "";
}

std::vector<CXCursor> getBinaryOperands(CXCursor cursor) {
    std::vector<CXCursor> operands;
    for (auto child : getChildren(cursor)) {
        while (clang_isUnexposed(child.kind)) {
            child = getChildren(child).at(0);
        }
        operands.push_back(child);
    }
    return operands;
}

std::vector<CXCursor> getLocalVars(CXCursor cursor) {
    std::vector<CXCursor> vars;
    for (auto child : getDescendants(cursor)) {
        if (clang_getCursorKind(child) == CXCursor_VarDecl) {
            vars.push_back(child);
        }
    }
    return vars;
}

int getIntegerLiteral(CXCursor cursor) {
    CXSourceRange range = clang_getCursorExtent(cursor);
    CXToken* tokens = nullptr;
    unsigned int numTokens = 0;
    CXTranslationUnit TU = clang_Cursor_getTranslationUnit(cursor);
    clang_tokenize(TU, range, &tokens, &numTokens);
    if (numTokens > 0) {
        CXString tokenSpelling = clang_getTokenSpelling(TU, tokens[0]);
        std::string literal = clang_getCString(tokenSpelling);
        clang_disposeString(tokenSpelling);
        clang_disposeTokens(TU, tokens, numTokens);
        return std::stoi(literal);
    }
    clang_disposeTokens(TU, tokens, numTokens);
    return 0;
}

std::string getSymbolName(CXCursor cursor) {
    if (cursor.kind == CXCursor_DeclRefExpr) {
        CXCursor definition = clang_getCursorDefinition(cursor);
        return getSymbolName(definition);
    }
    if (cursor.kind == CXCursor_FunctionDecl) {
        CXString funcName = clang_getCursorSpelling(cursor);
        std::string funcNameStr = clang_getCString(funcName);
        clang_disposeString(funcName);
        return funcNameStr;
    } else if (cursor.kind == CXCursor_CallExpr) {
        CXCursor definition = clang_getCursorDefinition(cursor);
        return getSymbolName(definition);
    }
    CXString name = clang_getCursorSpelling(cursor);
    std::string varName = clang_getCString(name);
    clang_disposeString(name);

    CXCursor parent = clang_getCursorSemanticParent(cursor);
    CXString pname = clang_getCursorSpelling(parent);
    std::string parentName = clang_getCString(pname);
    clang_disposeString(pname);

    return parentName + "::" + varName;
}

/// @brief Pushes the value in the given register onto the stack (stack grows downwards)
/// @param reg The register to push
void push(int reg) {
    emit("addi r" + esp + ", r" + esp + ", -4");
    emit("st r" + std::to_string(reg) + ", 0(r" + esp + ") ; push");
}

/// @brief Pops the top value off the stack into the given register (stack grows downwards)
/// @param reg The register to pop into
void pop(int reg) {
    emit("ld r" + std::to_string(reg) + ", 0(r" + esp + ") ; pop");
    emit("addi r" + esp + ", r" + esp + ", 4");
    // clear stack
    Register reg1;
    emit("la r" + std::to_string(reg1) + ", 0");
    emit("st r" + std::to_string(reg1) + ", -4(r" + esp + ")");
}

void pop(std::string reg) { pop(std::stoi(reg)); }

void push(std::string reg) { push(std::stoi(reg)); }

void call(std::string funcName) {
    Register reg;
    emit("brlnv r" + std::to_string(reg));
    emit("addi r" + std::to_string(reg) + ", r" + std::to_string(reg) + ", 20");  // Account for return address + instructions
    push(reg);  // Push return address
    emit("la r" + std::to_string(reg) + ", " + symTab[funcName].label);
    emit("br r" + std::to_string(reg) + " ; call " + funcName);
}

void ret() {
    Register reg;
    pop(reg);
    emit("br r" + std::to_string(reg) + " ; ret");
}

void iadd() {
    // pop top two operands off stack and add them, then push
    Register reg1, reg2;
    pop(reg1);
    pop(reg2);
    // TODO hard code stack offsets instead of using pop()

    emit("add r" + std::to_string(reg1) + ", r" + std::to_string(reg1) + ", r" +
         std::to_string(reg2));
    push(reg1);
}

void isub() {
    // pop top two operands off stack and add them, then push
    Register reg1, reg2;
    pop(reg1);  // TODO hard code stack offsets instead of using pop()
    pop(reg2);
    emit("sub r" + std::to_string(reg1) + ", r" + std::to_string(reg1) + ", r" +
         std::to_string(reg2));
    push(reg1);
}

void ineg() {
    Register reg;
    pop(reg);
    emit("neg r" + std::to_string(reg) + ", r" + std::to_string(reg));
}

void ior() {
    Register reg1, reg2;
    pop(reg1);
    pop(reg2);
    emit("or r" + std::to_string(reg1) + ", r" + std::to_string(reg1) + ", r" +
         std::to_string(reg2));
    push(reg1);
}

void iand() {
    Register reg1, reg2;
    pop(reg1);
    pop(reg2);
    emit("and r" + std::to_string(reg1) + ", r" + std::to_string(reg1) + ", r" +
         std::to_string(reg2));
    push(reg1);
}

void inot() {
    Register reg;
    pop(reg);
    emit("not r" + std::to_string(reg) + ", r" + std::to_string(reg));
    push(reg);
}

void add2(SymTabEntry sym1, SymTabEntry sym2) {
    if(sym1.kind == SymTabEntry::Kind::param) {
        emit("ld r" + std::to_string(sym1.reg) + ", " + std::to_string(sym1.offset) + "(r" + ebp + ")");
    } else if(sym1.kind == SymTabEntry::Kind::local){
        emit("ld r" + std::to_string(sym1.reg) + ", -" + std::to_string(sym1.offset) + "(r" + ebp + ")");
    } 
    if(sym2.kind == SymTabEntry::Kind::param) {
        emit("ld r" + std::to_string(sym2.reg) + ", " + std::to_string(sym2.offset) + "(r" + ebp + ")");
    } else if(sym2.kind == SymTabEntry::Kind::local){
        emit("ld r" + std::to_string(sym2.reg) + ", -" + std::to_string(sym2.offset) + "(r" + ebp + ")");
    }

    emit("add r" + eax + ", r" + std::to_string(sym1.reg) + ", r" + std::to_string(sym2.reg) + " ; add ");
}

void visitCompute(CXCursor cursor) {
    if (cursor.kind == CXCursor_BinaryOperator) {
        std::string op = getBinaryOperator(cursor);
        std::vector<CXCursor> operands = getBinaryOperands(cursor);
        std::string lhsNameStr = getSymbolName(operands.at(0));
        std::string rhsNameStr = getSymbolName(operands.at(1));
        if (op == "+") {
            add2(symTab[lhsNameStr], symTab[rhsNameStr]);
            // } else if (op == "-") {
            //     isub();
            // } else if (op == "|") {
            //     ior();
            // } else if (op == "&") {
            //     iand();
        } else {
            std::cerr << "Error: Unknown Binary Operator " << op << std::endl;
        }
        // } else if (cursor.kind == CXCursor_UnaryOperator) {
        //     std::string op = getBinaryOperator(cursor);
        //     if (op == "-") {
        //         ineg();
        //     } else if (op == "~") {
        //         inot();
        //     } else {
        //         std::cerr << "Error: Unknown Unary Operator " << op << std::endl;
        //     }
    } else {
        std::cerr << "Error: Unknown Compute Operator " << cursor << std::endl;
    }
}

void visitFunctionDecl(CXCursor cursor) {
    // get params
    std::vector<CXCursor> params = getParams(cursor);
    std::vector<CXCursor> locals = getLocalVars(cursor);

    std::string funcName = getSymbolName(cursor);

    SymTabEntry& sym = symTab[funcName];
    sym.kind = SymTabEntry::Kind::func;
    sym.label = "func" + std::to_string(labelNum++);
    sym.offset = locals.size() - 1;  // TODO hacky

    emit(sym.label + ": ; " + funcName);

    int requiredStack = (params.size() + locals.size()) * 4;

    // Prologue
    push(ebp);
    emit("addi r" + ebp + ", r" + esp + ", 0");  // ebp = esp
    // Allcate space for locals (assuming 4 bytes each)
    emit("addi r" + esp + ", r" + esp + ", -" + std::to_string(requiredStack));

    // visit params
    for (int i = 0; i < params.size(); i++) {
        std::string nameStr = getSymbolName(params[i]);
        SymTabEntry& sym = symTab[nameStr];
        sym.offset = (i + 2) * 4;  // TODO hacky
        sym.kind = SymTabEntry::Kind::param;
        // emit("st r" + std::to_string(i + RESERVED_REGS) + ", " + std::to_string(sym.offset) + "(r" +
        //      ebp + ")");

        visit(params[i]);
    }

    // visit body
    for (auto child : getBody(cursor)) {
        visit(child);
    }

    // return epilogue
    emit("addi r" + esp + ", r" + ebp + ", 0");  // esp = ebp
    pop(ebp);
    ret();
}

void visitVarDecl(CXCursor cursor) {
    std::string nameStr = getSymbolName(cursor);

    SymTabEntry& sym = symTab[nameStr];

    if (sym.offset == -1) {  // Not yet allocated
        // Calculate offset
        CXCursor parent = clang_getCursorSemanticParent(cursor);
        SymTabEntry& parentSym = symTab[getSymbolName(parent)];
        sym.offset = parentSym.offset * 4;
        parentSym.offset--;

        sym.kind = SymTabEntry::Kind::local;

        std::cout << ";alloc " << nameStr << " to offset" << sym.offset << std::endl;
    }
}

void visitIntegerLiteral(CXCursor cursor) {
    int value = getIntegerLiteral(cursor);
    emit("la r" + eax + ", " + std::to_string(value));
}

void visitDeclRef(CXCursor cursor) {
    std::string nameStr = getSymbolName(cursor);

    SymTabEntry sym = symTab[nameStr];

    if (sym.reg == -1) {
        std::cerr << "Error: " << nameStr << " not declared" << std::endl;
        return;
    }

    if(sym.kind == SymTabEntry::Kind::param) {
        emit("ld r" + eax + ", " + std::to_string(sym.offset) + "(r" + ebp + ") ; " + nameStr);
    } else if(sym.kind == SymTabEntry::Kind::local){
        emit("ld r" + eax + ", -" + std::to_string(sym.offset) + "(r" + ebp + ") ; " + nameStr);
    } else {
        std::cerr << "Error: " << nameStr << " not a local or param" << std::endl;
    }
}

void visitAssign(CXCursor cursor) {
    std::vector<CXCursor> children = getChildren(cursor);
    CXCursor lhs = children.at(0);
    CXCursor rhs = children.at(1);

    std::string lhsNameStr = getSymbolName(lhs);

    SymTabEntry lhsSym = symTab[lhsNameStr];

    if (lhsSym.reg == -1) {
        std::cerr << "Error: LHS " << lhsNameStr << " not declared" << std::endl;
        return;
    }

    visit(rhs);  // Puts result in eax

    emit("st r" + eax + ", -" + std::to_string(lhsSym.offset) + "(r" + ebp + ") ; " + lhsNameStr);
}

void visitInvoke(CXCursor cursor) {
    std::string nameStr = getSymbolName(cursor);
    SymTabEntry sym = symTab[nameStr];

    if (sym.label == "") {
        std::cerr << "Error: Invoke " << nameStr << " not declared" << std::endl;
        return;
    }

    std::vector<CXCursor> args = getArgs(cursor);

    for (auto arg : args) {
        visit(arg);
        push(eax);
    }

    call(nameStr);
}

void visit(CXCursor cursor) {
    CXCursorKind kind = clang_getCursorKind(cursor);

    if (clang_isDeclaration(kind)) {
        if (kind == CXCursor_FunctionDecl) visitFunctionDecl(cursor);
        if (kind == CXCursor_VarDecl) visitVarDecl(cursor);
    } else if (kind == CXCursor_BinaryOperator) {
        std::string op = getBinaryOperator(cursor);
        if (op != "") {
            if (op == "=") {
                visitAssign(cursor);
            } else {
                visitCompute(cursor);
            }
        }
    } else if (kind == CXCursor_CallExpr) {
        visitInvoke(cursor);
    } else if (kind == CXCursor_IntegerLiteral) {
        visitIntegerLiteral(cursor);
    } else if (kind == CXCursor_DeclRefExpr) {
        visitDeclRef(cursor);
    } else {
        for (auto child : getChildren(cursor)) {
            visit(child);
        }
    }
}

CXChildVisitResult printVisitor(CXCursor cursor, CXCursor parent, CXClientData clientData) {
    int depth = *(int*)clientData;
    int nextDepth = depth + 1;

    std::cout << std::string(depth, '-') << cursor << std::endl;

    clang_visitChildren(cursor, printVisitor, &nextDepth);

    return CXChildVisit_Continue;
}

void setupAsm() {
    // Setup stack
    emit(".org " + std::to_string(stackBegin));
    emit("STACK: .dw " + std::to_string(stackSize / 4));
    emit(".org 0");
    emit("la r" + esp + ", STACK");
    emit("la r" + ebp + ", STACK");

    // Create main stack frame
    Register reg;
    emit("la r" + std::to_string(reg) + ", END ; main exit");
    push(reg);  // Push return address
    SymTabEntry sym = symTab[ENTRY_POINT];
    emit("la r" + std::to_string(reg) + ", " + "func1");  // TODO: fix this
    emit("br r" + std::to_string(reg) + " ; call " + ENTRY_POINT);

    for(int i = 0; i < RESERVED_REGS; i++) {
        symTab["unused" + std::to_string(i)]; // Reserve registers, needed?
    }

}

auto main(int argc, const char** argv) -> int {
    const char* fileName = "test.c";

    CXIndex index = clang_createIndex(0, 0);
    CXTranslationUnit translationUnit =
        clang_parseTranslationUnit(index, fileName, nullptr, 0, nullptr, 0, CXTranslationUnit_None);

    if (!translationUnit) {
        std::cerr << "Error parsing translation unit" << std::endl;
        return 1;
    }

    CXCursor rootCursor = clang_getTranslationUnitCursor(translationUnit);

    if (argc > 1) {
        std::string arg = argv[1];
        if (arg == "-ast") {
            int startingDepth = 0;
            clang_visitChildren(rootCursor, printVisitor, &startingDepth);
            return 0;
        } else {
            std::cerr << "Usage: " << argv[0] << " [-ast]" << std::endl;
            return 1;
        }
    }

    setupAsm();

    visit(rootCursor);

    Register reg;
    emit("stop");
    emit("la r" + std::to_string(reg) + ", END");
    emit("END:");
    emit("br r" + std::to_string(reg));  // End in infinite loop

    clang_disposeTranslationUnit(translationUnit);
    clang_disposeIndex(index);

    return 0;
}
