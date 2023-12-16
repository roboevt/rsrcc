#include <clang-c/Index.h>

#include <iostream>
#include <string>
#include <vector>

std::ostream& operator<<(std::ostream& stream, const CXString& str);

std::ostream& operator<<(std::ostream& stream, const CXCursor& cursor);

CXChildVisitResult visitor(CXCursor cursor, CXCursor parent, CXClientData clientData);

std::vector<CXCursor> getChildren(CXCursor cursor);

std::vector<CXCursor> getParams(CXCursor cursor);

std::string getBinaryOperator(CXCursor cursor);

int getIntegerLiteral(CXCursor cursor);

void visit(CXCursor cursor);

void visitFunctionDecl(CXCursor cursor);

void visitVarDecl(CXCursor cursor);

void visitAssign(CXCursor cursor);

void visitCompute(CXCursor cursor);

void setupAsm();