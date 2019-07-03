#include "../common/tutorial/tutorial.h"

namespace embree
{
struct Tutorial : public SceneLoadingTutorialApplication
{
    Tutorial()
        : SceneLoadingTutorialApplication("viewer",FEATURE_RTCORE) {}

    void postParseCommandLine()
    {
        /* load default scene if none specified */
        if (scene->size() == 0 && sceneFilename.ext() == "") {
            FileName file =
                FileName::executableFolder() + FileName("models/cornell_box.ecs");
            parseCommandLine(
                new ParseStream(new LineCommentFilter(file, "#")), file.path());
        }
    }
};

}

int main(int argc, char** argv) {
  return embree::Tutorial().main(argc,argv);
}
