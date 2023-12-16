// g++ -o rsrcc rsrcc.cpp -I /usr/lib/llvm-14/include/ -L /usr/lib/llvm-14/lib/ -lclang

#include "rsrcc.h"

#include <unordered_map>

static const std::string rsp = "1";
static const std::string rret = "2";

static constexpr int stackBegin = 4096;
static constexpr int stackSize = 4096;

struct SymTabEntry {
    int reg;

    enum class Kind { var, func, invalid } kind;

    std::string label;

    SymTabEntry() : reg(-1), kind(Kind::invalid) {}
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
        case CXCursor_BinaryOperator: {
            stream << getBinaryOperator(cursor);
            break;
        case CXCursor_IntegerLiteral: {
                stream << getIntegerLiteral(cursor);
                break;
                default:
                    stream << clang_getCursorSpelling(cursor);
            }
        }
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

std::vector<CXCursor> getParams(CXCursor cursor) {
    std::vector<CXCursor> params;
    for (auto child : getChildren(cursor)) {
        if (clang_getCursorKind(child) == CXCursor_ParmDecl) {
            params.push_back(child);
        }
    }
    return params;
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

/// @brief Pushes the value in the given register onto the stack (stack grows downwards)
/// @param reg The register to push
void push(int reg) {
    emit("addi r" + rsp + ", r" + rsp + ", -4");
    emit("st r" + std::to_string(reg) + ", 0(r" + rsp + ")");
}

/// @brief Pops the top value off the stack into the given register (stack grows downwards)
/// @param reg The register to pop into
void pop(int reg) {
    emit("ld r" + std::to_string(reg) + ", 0(r" + rsp + ")");
    emit("addi r" + rsp + ", r" + rsp + ", 4");
}

void iadd() {
    // pop top two operands off stack and add them, then push
    int rscratch = symTab.size();
    pop(rscratch);  // TODO hard code stack offsets instead of using pop()
    pop(rscratch + 1);
    emit("add r" + std::to_string(rscratch) + ", r" + std::to_string(rscratch) + ", r" +
         std::to_string(rscratch + 1));
    push(rscratch);
}

void visitFunctionDecl(CXCursor cursor) {
    // get params
    std::vector<CXCursor> params = getParams(cursor);

    CXString name = clang_getCursorSpelling(cursor);
    std::string nameStr = clang_getCString(name);
    clang_disposeString(name);

    SymTabEntry sym = symTab[nameStr];
    sym.kind = SymTabEntry::Kind::func;
    sym.label = "func" + std::to_string(labelNum++);
    symTab[nameStr] = sym;

    emit(sym.label + ":");

    // print params
    // std::cout << "params: ";
    // for (auto param : params) {
    //     std::cout << param << " ";
    //     visitVarDecl(param);
    // }
    // std::cout << std::endl;

    // visit body
    for (auto child : getChildren(cursor)) {
        visit(child);
    }

    // return
    emit("br r" + rret);  // TODO multiple returns (recursion)
}

void visitVarDecl(CXCursor cursor) {
    CXString name = clang_getCursorSpelling(cursor);
    std::string nameStr = clang_getCString(name);
    clang_disposeString(name);

    SymTabEntry sym = symTab[nameStr];

    if (sym.reg == -1) {
        sym.reg = symTab.size();
        symTab[nameStr] = sym;
    }
}

void visitIntegerLiteral(CXCursor cursor) {
    int value = getIntegerLiteral(cursor);
    int rscratch = symTab.size();
    emit("addi r" + std::to_string(rscratch) + ", r0, " + std::to_string(value));
    push(rscratch);
}

void visitAssign(CXCursor cursor) {
    std::vector<CXCursor> children = getChildren(cursor);
    CXCursor lhs = children.at(0);
    CXCursor rhs = children.at(1);

    CXString lhsName = clang_getCursorSpelling(lhs);
    std::string lhsNameStr = clang_getCString(lhsName);
    clang_disposeString(lhsName);

    SymTabEntry lhsSym = symTab[lhsNameStr];

    if (lhsSym.reg == -1) {
        std::cerr << "Error: LHS " << lhsNameStr << " not declared" << std::endl;
        return;
    }

    visit(rhs);

    // Now we should have RHS on the stack.
    pop(lhsSym.reg);
}

void visitCompute(CXCursor cursor) {
    std::string op = getBinaryOperator(cursor);
    if (op == "+") {
        iadd();
    }
}

void visitInvoke(CXCursor cursor) {
    CXString name = clang_getCursorSpelling(cursor);
    std::string nameStr = clang_getCString(name);
    clang_disposeString(name);

    SymTabEntry sym = symTab[nameStr];

    if (sym.label == "") {
        std::cerr << "Error: Invoke " << nameStr << " not declared" << std::endl;
        return;
    }

    std::vector<CXCursor> params = getParams(cursor);

    for (auto param : params) {
        visit(param);
    }

    int rscratch = symTab.size();

    emit("la r" + std::to_string(rscratch) + ", " + sym.label);
    emit("brl r" + rsp + ", r" +
         std::to_string(rscratch));  // Branch with linkage (TODO multiple calls (recursion))
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
    }
}

CXChildVisitResult visitor(CXCursor cursor, CXCursor parent, CXClientData clientData) {
    int depth = *(int*)clientData;
    int nextDepth = depth + 1;

    // std::cout << std::string(depth, '-') << cursor << std::endl;

    visit(cursor);

    clang_visitChildren(cursor, visitor, &nextDepth);

    return CXChildVisit_Continue;
}

void setupAsm() {
    emit(".org " + std::to_string(stackBegin));
    emit("STACK: .dw " + std::to_string(stackSize/4));
    emit("la r" + rsp + ", STACK");
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

    // Perform operations with the translation unit...

    CXCursor rootCursor = clang_getTranslationUnitCursor(translationUnit);

    int startingDepth = 0;

    setupAsm();

    clang_visitChildren(rootCursor, visitor, &startingDepth);

    clang_disposeTranslationUnit(translationUnit);
    clang_disposeIndex(index);

    return 0;
}
